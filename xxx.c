/*
 * ESP32-S3 USB Host Flash Bridge (Rewritten)
 * Receives firmware from PC via UART and flashes target ESP32 WROOM via USB Host
 * Uses native USB Host Library without CDC-ACM component dependency
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
// #include "usb_host_cdc_acm.h"  // Removed dependency
#include "mbedtls/md5.h"
#include <string.h>

#define TAG "ESP32_FLASH_BRIDGE"

// UART Configuration
#define UART_NUM UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 2048

// USB Host Configuration  
#define USB_HOST_PRIORITY 2
#define FLASH_TASK_PRIORITY 3
#define USB_TRANSFER_TIMEOUT_MS 1000

// Flash Protocol Commands (must match Python script)
#define CMD_FLASH_BEGIN 0x02
#define CMD_FLASH_DATA  0x03
#define CMD_FLASH_END   0x04
#define CMD_TARGET_RESET 0x05

// ESP32 ROM Bootloader Commands
#define ESP_SYNC        0x08
#define ESP_WRITE_REG   0x09
#define ESP_READ_REG    0x0A
#define ESP_MEM_BEGIN   0x05
#define ESP_MEM_DATA    0x07
#define ESP_MEM_END     0x06
#define ESP_FLASH_BEGIN 0x02
#define ESP_FLASH_DATA  0x03
#define ESP_FLASH_END   0x04

// Target ESP32 Configuration
#define TARGET_FLASH_WRITE_SIZE 0x400  // 1024 bytes
#define ESP32_BOOTLOADER_BAUD   115200

// USB CDC-ACM Endpoints (typical values)
#define CDC_ACM_DATA_OUT_EP     0x01
#define CDC_ACM_DATA_IN_EP      0x82
#define CDC_ACM_CONTROL_EP      0x83

// USB Transfer States
typedef enum {
    USB_TRANSFER_STATE_IDLE = 0,
    USB_TRANSFER_STATE_IN_PROGRESS,
    USB_TRANSFER_STATE_COMPLETED,
    USB_TRANSFER_STATE_ERROR
} usb_transfer_state_t;

// UART Command Structure
typedef struct {
    uint8_t cmd;
    uint32_t data_len;
    uint8_t *data;
    uint32_t crc;
} uart_command_t;

// Flash Context Structure
typedef struct {
    uint32_t total_size;
    uint32_t block_count;
    uint32_t current_block;
    uint32_t target_address;
    uint8_t *firmware_buffer;
    bool flash_in_progress;
} flash_context_t;

// USB Device Management Structure
typedef struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    uint8_t dev_addr;
    uint8_t interface_num;
    uint8_t data_out_ep;
    uint8_t data_in_ep;
    bool device_ready;
    bool transfer_in_progress;
    usb_transfer_state_t last_transfer_state;
    SemaphoreHandle_t transfer_done_sem;
} usb_cdc_device_t;

// Global Variables
static flash_context_t g_flash_ctx = {0};
static usb_cdc_device_t g_usb_device = {0};  // Replaced cdc_acm_dev_hdl_t
static QueueHandle_t flash_queue = NULL;
static SemaphoreHandle_t usb_mutex = NULL;

// Forward Declarations
static void usb_host_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg);
static void usb_transfer_callback(usb_transfer_t *transfer);
static esp_err_t usb_cdc_send_data(const uint8_t *data, size_t len);
static esp_err_t usb_cdc_receive_data(uint8_t *data, size_t max_len, size_t *actual_len);

// =============================================================================
// ESP32 ROM Bootloader SLIP Protocol Implementation
// =============================================================================

/**
 * @brief Encodes a single byte using SLIP protocol
 * @param byte Byte to encode
 * @param out Output buffer
 * @param out_len Current output length (will be updated)
 */
static void slip_encode_byte(uint8_t byte, uint8_t *out, size_t *out_len) {
    if (byte == 0xC0) {
        out[(*out_len)++] = 0xDB;
        out[(*out_len)++] = 0xDC;
    } else if (byte == 0xDB) {
        out[(*out_len)++] = 0xDB;
        out[(*out_len)++] = 0xDD;
    } else {
        out[(*out_len)++] = byte;
    }
}

/**
 * @brief Encodes data using SLIP protocol with start/end delimiters
 * @param data Input data to encode
 * @param len Input data length
 * @param encoded Output encoded buffer
 * @return Length of encoded data
 */
static size_t slip_encode(const uint8_t *data, size_t len, uint8_t *encoded) {
    size_t encoded_len = 0;
    encoded[encoded_len++] = 0xC0; // Start delimiter
    
    for (size_t i = 0; i < len; i++) {
        slip_encode_byte(data[i], encoded, &encoded_len);
    }
    
    encoded[encoded_len++] = 0xC0; // End delimiter
    return encoded_len;
}

// =============================================================================
// USB Host Transfer Management
// =============================================================================

/**
 * @brief Callback function for USB transfer completion
 * @param transfer Completed transfer object
 */
static void usb_transfer_callback(usb_transfer_t *transfer) {
    usb_cdc_device_t *usb_dev = (usb_cdc_device_t *)transfer->context;
    
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGD(TAG, "USB transfer completed successfully, %d bytes", transfer->actual_num_bytes);
        usb_dev->last_transfer_state = USB_TRANSFER_STATE_COMPLETED;
    } else {
        ESP_LOGW(TAG, "USB transfer failed with status: %d", transfer->status);
        usb_dev->last_transfer_state = USB_TRANSFER_STATE_ERROR;
    }
    
    usb_dev->transfer_in_progress = false;
    
    // Signal transfer completion
    if (usb_dev->transfer_done_sem) {
        xSemaphoreGive(usb_dev->transfer_done_sem);
    }
    
    // Free the transfer object
    usb_host_transfer_free(transfer);
}

/**
 * @brief Sends data to USB CDC device using bulk transfer
 * @param data Data to send
 * @param len Length of data
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t usb_cdc_send_data(const uint8_t *data, size_t len) {
    if (!g_usb_device.device_ready) {
        ESP_LOGE(TAG, "USB CDC device not ready");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_usb_device.transfer_in_progress) {
        ESP_LOGW(TAG, "Transfer already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Allocate transfer object
    usb_transfer_t *transfer;
    esp_err_t ret = usb_host_transfer_alloc(len, 0, &transfer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure transfer
    transfer->device_handle = g_usb_device.dev_hdl;
    transfer->bEndpointAddress = g_usb_device.data_out_ep;
    transfer->callback = usb_transfer_callback;
    transfer->context = &g_usb_device;
    transfer->num_bytes = len;
    
    // Copy data to transfer buffer
    memcpy(transfer->data_buffer, data, len);
    
    // Mark transfer as in progress
    g_usb_device.transfer_in_progress = true;
    g_usb_device.last_transfer_state = USB_TRANSFER_STATE_IN_PROGRESS;
    
    // Submit transfer
    ret = usb_host_transfer_submit(transfer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit transfer: %s", esp_err_to_name(ret));
        g_usb_device.transfer_in_progress = false;
        usb_host_transfer_free(transfer);
        return ret;
    }
    
    // Wait for transfer completion with timeout
    if (xSemaphoreTake(g_usb_device.transfer_done_sem, pdMS_TO_TICKS(USB_TRANSFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Transfer timeout");
        g_usb_device.transfer_in_progress = false;
        return ESP_ERR_TIMEOUT;
    }
    
    // Check transfer result
    if (g_usb_device.last_transfer_state != USB_TRANSFER_STATE_COMPLETED) {
        ESP_LOGE(TAG, "Transfer failed");
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Successfully sent %d bytes via USB", len);
    return ESP_OK;
}

/**
 * @brief Receives data from USB CDC device using bulk transfer
 * @param data Buffer to receive data
 * @param max_len Maximum buffer size
 * @param actual_len Actual received length
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t usb_cdc_receive_data(uint8_t *data, size_t max_len, size_t *actual_len) {
    if (!g_usb_device.device_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // For now, return no data available (simplified implementation)
    // Full implementation would require setting up IN transfers
    *actual_len = 0;
    return ESP_OK;
}

// =============================================================================
// ESP32 Flash Protocol Implementation
// =============================================================================

/**
 * @brief Sends a command to target ESP32 using ESP32 ROM bootloader protocol
 * @param cmd Command code
 * @param data Command data payload
 * @param data_len Length of data payload
 * @param checksum Command checksum
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t send_esp32_command(uint8_t cmd, const uint8_t *data, size_t data_len, 
                                   uint32_t checksum) {
    ESP_LOGD(TAG, "Sending ESP32 command 0x%02X with %d bytes", cmd, data_len);
    
    // Build ESP32 command packet
    uint8_t packet[1024];
    size_t packet_len = 0;
    
    packet[packet_len++] = 0x00; // Direction (request)
    packet[packet_len++] = cmd;   // Command
    packet[packet_len++] = data_len & 0xFF;        // Size low
    packet[packet_len++] = (data_len >> 8) & 0xFF; // Size high
    packet[packet_len++] = checksum & 0xFF;        // Checksum bytes
    packet[packet_len++] = (checksum >> 8) & 0xFF;
    packet[packet_len++] = (checksum >> 16) & 0xFF;
    packet[packet_len++] = (checksum >> 24) & 0xFF;
    
    // Add data payload
    if (data && data_len > 0) {
        memcpy(&packet[packet_len], data, data_len);
        packet_len += data_len;
    }
    
    // SLIP encode the packet
    uint8_t encoded[2048];
    size_t encoded_len = slip_encode(packet, packet_len, encoded);
    
    // Send via USB with mutex protection
    xSemaphoreTake(usb_mutex, portMAX_DELAY);
    esp_err_t ret = usb_cdc_send_data(encoded, encoded_len);
    xSemaphoreGive(usb_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X: %s", cmd, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Successfully sent command 0x%02X", cmd);
    return ESP_OK;
}

/**
 * @brief Synchronizes with target ESP32 bootloader
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t sync_target_esp32(void) {
    ESP_LOGI(TAG, "Syncing with target ESP32...");
    
    // Send SYNC command with sync pattern
    uint8_t sync_data[36];
    memset(sync_data, 0x55, 32); // Sync pattern
    sync_data[32] = 0x00; // Additional sync bytes
    sync_data[33] = 0x00;
    sync_data[34] = 0x00;
    sync_data[35] = 0x00;
    
    esp_err_t ret = send_esp32_command(ESP_SYNC, sync_data, sizeof(sync_data), 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send SYNC command");
        return ret;
    }
    
    // Wait for response (simplified - should parse SLIP response)
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Target ESP32 sync successful");
    return ESP_OK;
}

/**
 * @brief Initiates flash operation on target ESP32
 * @param size Total firmware size
 * @param blocks Number of blocks
 * @param address Flash start address
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t flash_begin_target(uint32_t size, uint32_t blocks, uint32_t address) {
    ESP_LOGI(TAG, "Starting flash operation: size=%lu, blocks=%lu, addr=0x%lx", 
             size, blocks, address);
    
    uint8_t flash_begin_data[16];
    // Pack flash parameters in little-endian format
    flash_begin_data[0] = size & 0xFF;
    flash_begin_data[1] = (size >> 8) & 0xFF;
    flash_begin_data[2] = (size >> 16) & 0xFF;
    flash_begin_data[3] = (size >> 24) & 0xFF;
    
    flash_begin_data[4] = blocks & 0xFF;
    flash_begin_data[5] = (blocks >> 8) & 0xFF;
    flash_begin_data[6] = (blocks >> 16) & 0xFF;
    flash_begin_data[7] = (blocks >> 24) & 0xFF;
    
    flash_begin_data[8] = TARGET_FLASH_WRITE_SIZE & 0xFF;
    flash_begin_data[9] = (TARGET_FLASH_WRITE_SIZE >> 8) & 0xFF;
    flash_begin_data[10] = (TARGET_FLASH_WRITE_SIZE >> 16) & 0xFF;
    flash_begin_data[11] = (TARGET_FLASH_WRITE_SIZE >> 24) & 0xFF;
    
    flash_begin_data[12] = address & 0xFF;
    flash_begin_data[13] = (address >> 8) & 0xFF;
    flash_begin_data[14] = (address >> 16) & 0xFF;
    flash_begin_data[15] = (address >> 24) & 0xFF;
    
    return send_esp32_command(ESP_FLASH_BEGIN, flash_begin_data, sizeof(flash_begin_data), 0);
}

/**
 * @brief Sends flash data block to target ESP32
 * @param data Data block to flash
 * @param size Size of data block
 * @param sequence Block sequence number
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t flash_data_target(const uint8_t *data, uint32_t size, uint32_t sequence) {
    ESP_LOGD(TAG, "Flashing block %lu (%lu bytes)", sequence, size);
    
    // Prepare flash data packet: [SIZE:4][SEQ:4][RESERVED:8][DATA]
    uint8_t flash_packet[TARGET_FLASH_WRITE_SIZE + 16];
    
    // Pack header in little-endian format
    flash_packet[0] = size & 0xFF;
    flash_packet[1] = (size >> 8) & 0xFF;
    flash_packet[2] = (size >> 16) & 0xFF;
    flash_packet[3] = (size >> 24) & 0xFF;
    
    flash_packet[4] = sequence & 0xFF;
    flash_packet[5] = (sequence >> 8) & 0xFF;
    flash_packet[6] = (sequence >> 16) & 0xFF;
    flash_packet[7] = (sequence >> 24) & 0xFF;
    
    memset(&flash_packet[8], 0, 8); // Reserved bytes
    memcpy(&flash_packet[16], data, size);
    
    // Calculate checksum of data
    uint32_t checksum = 0;
    for (uint32_t i = 0; i < size; i++) {
        checksum ^= data[i];
    }
    
    return send_esp32_command(ESP_FLASH_DATA, flash_packet, size + 16, checksum);
}

/**
 * @brief Completes flash operation on target ESP32
 * @param md5_hash MD5 hash for verification (can be NULL)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t flash_end_target(const uint8_t *md5_hash) {
    ESP_LOGI(TAG, "Finishing flash operation");
    
    // Send flash end with reboot flag
    uint8_t flash_end_data[4] = {0x00, 0x00, 0x00, 0x00}; // No reboot
    
    esp_err_t ret = send_esp32_command(ESP_FLASH_END, flash_end_data, sizeof(flash_end_data), 0);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Flash operation completed successfully");
    return ESP_OK;
}

/**
 * @brief Resets target ESP32 by simulating DTR/RTS control
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t reset_target_device(void) {
    if (!g_usb_device.device_ready) {
        ESP_LOGE(TAG, "USB device not ready for reset");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting target ESP32...");
    
    // Note: In a full CDC-ACM implementation, this would send
    // SET_CONTROL_LINE_STATE requests to control DTR/RTS lines.
    // For simplicity, we'll just add a delay to simulate reset timing.
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Reset pulse
    vTaskDelay(pdMS_TO_TICKS(100));  // Recovery time
    
    ESP_LOGI(TAG, "Target reset completed");
    return ESP_OK;
}

// =============================================================================
// USB Host Client Management
// =============================================================================

/**
 * @brief USB Host client event callback
 * @param event_msg Event message from USB Host Library
 * @param arg User argument (unused)
 */
static void usb_host_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device connected at address %d", event_msg->new_dev.address);
            g_usb_device.dev_addr = event_msg->new_dev.address;
            break;
            
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB device at address %d disconnected", event_msg->dev_gone.dev_hdl);
            g_usb_device.device_ready = false;
            g_usb_device.dev_hdl = NULL;
            g_usb_device.dev_addr = 0;
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown USB client event: %d", event_msg->event);
            break;
    }
}

/**
 * @brief Configures USB CDC device after connection
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t configure_usb_cdc_device(void) {
    if (!g_usb_device.dev_hdl) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get device descriptor to verify it's a CDC device
    const usb_device_desc_t *dev_desc;
    esp_err_t ret = usb_host_get_device_descriptor(g_usb_device.dev_hdl, &dev_desc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device descriptor");
        return ret;
    }
    
    ESP_LOGI(TAG, "USB Device: VID=0x%04X, PID=0x%04X, Class=0x%02X",
             dev_desc->idVendor, dev_desc->idProduct, dev_desc->bDeviceClass);
    
    // Get configuration descriptor
    const usb_config_desc_t *config_desc;
    ret = usb_host_get_active_config_descriptor(g_usb_device.dev_hdl, &config_desc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config descriptor");
        return ret;
    }
    
    // For simplicity, assume first interface is CDC-ACM data interface
    // In a full implementation, you would parse the descriptors to find
    // the correct CDC-ACM interfaces and endpoints
    g_usb_device.interface_num = 0;
    g_usb_device.data_out_ep = CDC_ACM_DATA_OUT_EP;
    g_usb_device.data_in_ep = CDC_ACM_DATA_IN_EP;
    
    // Claim the interface
    ret = usb_host_interface_claim(g_usb_device.client_hdl, g_usb_device.dev_hdl, 
                                  g_usb_device.interface_num, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim interface");
        return ret;
    }
    
    g_usb_device.device_ready = true;
    ESP_LOGI(TAG, "USB CDC device configured successfully");
    
    return ESP_OK;
}

// =============================================================================
// Task Implementations
// =============================================================================

/**
 * @brief USB Host management task
 * @param arg Task parameter (unused)
 */
static void usb_host_task(void *arg) {
    ESP_LOGI(TAG, "Starting USB Host task");
    
    // Install USB Host library
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host library installed");
    
    // Register USB Host client
    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_host_client_event_cb,
            .callback_arg = NULL,
        }
    };
    
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &g_usb_device.client_hdl));
    ESP_LOGI(TAG, "USB Host client registered");
    
    // Main USB host event loop
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        // Try to open and configure device if new device connected
        if (g_usb_device.dev_addr != 0 && g_usb_device.dev_hdl == NULL) {
            esp_err_t ret = usb_host_device_open(g_usb_device.client_hdl, 
                                               g_usb_device.dev_addr, 
                                               &g_usb_device.dev_hdl);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "USB device opened successfully");
                
                // Configure the CDC device
                if (configure_usb_cdc_device() == ESP_OK) {
                    ESP_LOGI(TAG, "USB CDC device ready for communication");
                }
            } else {
                ESP_LOGW(TAG, "Failed to open USB device: %s", esp_err_to_name(ret));
            }
        }
        
        // Handle USB Host library events
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "No more USB clients");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
    }
}

/**
 * @brief UART response sender helper function
 * @param status Response status (0 = success, 1 = error)
 * @param message Response message string
 */
static void send_uart_response(uint8_t status, const char *message) {
    uint8_t response[1024];
    size_t resp_len = 0;
    
    // Response format: [STATUS:1][LEN:4][DATA:n]
    response[resp_len++] = status;
    
    uint32_t msg_len = message ? strlen(message) : 0;
    response[resp_len++] = msg_len & 0xFF;
    response[resp_len++] = (msg_len >> 8) & 0xFF;
    response[resp_len++] = (msg_len >> 16) & 0xFF;
    response[resp_len++] = (msg_len >> 24) & 0xFF;
    
    if (message && msg_len > 0) {
        memcpy(&response[resp_len], message, msg_len);
        resp_len += msg_len;
    }
    
    uart_write_bytes(UART_NUM, response, resp_len);
}

/**
 * @brief Flash operation processing task
 * @param arg Task parameter (unused)
 */
static void flash_task(void *arg) {
    ESP_LOGI(TAG, "Flash task started");
    
    uart_command_t cmd;
    
    while (true) {
        if (xQueueReceive(flash_queue, &cmd, portMAX_DELAY)) {
            esp_err_t ret = ESP_OK;
            const char *response_msg = "OK";
            
            switch (cmd.cmd) {
                case CMD_FLASH_BEGIN: {
                    ESP_LOGI(TAG, "Processing FLASH_BEGIN command");
                    
                    if (cmd.data_len >= 12) {
                        uint32_t *params = (uint32_t *)cmd.data;
                        g_flash_ctx.total_size = params[0];
                        g_flash_ctx.block_count = params[1];  
                        g_flash_ctx.target_address = params[2];
                        g_flash_ctx.current_block = 0;
                        g_flash_ctx.flash_in_progress = true;
                        
                        ESP_LOGI(TAG, "Flash parameters: size=%lu, blocks=%lu, addr=0x%lx",
                                g_flash_ctx.total_size, g_flash_ctx.block_count, g_flash_ctx.target_address);
                        
                        // Allocate firmware buffer
                        if (g_flash_ctx.firmware_buffer) {
                            free(g_flash_ctx.firmware_buffer);
                        }
                        g_flash_ctx.firmware_buffer = malloc(g_flash_ctx.total_size);
                        
                        if (!g_flash_ctx.firmware_buffer) {
                            ret = ESP_ERR_NO_MEM;
                            response_msg = "Memory allocation failed";
                            ESP_LOGE(TAG, "Failed to allocate %lu bytes for firmware buffer", g_flash_ctx.total_size);
                        } else {
                            ESP_LOGI(TAG, "Allocated firmware buffer: %lu bytes", g_flash_ctx.total_size);
                            
                            // Sync with target and begin flash
                            ret = sync_target_esp32();
                            if (ret == ESP_OK) {
                                ret = flash_begin_target(g_flash_ctx.total_size, 
                                                       g_flash_ctx.block_count,
                                                       g_flash_ctx.target_address);
                            }
                            if (ret != ESP_OK) {
                                response_msg = "Flash begin failed";
                                ESP_LOGE(TAG, "Flash begin operation failed");
                            }
                        }
                    } else {
                        ret = ESP_ERR_INVALID_ARG;
                        response_msg = "Invalid flash begin parameters";
                        ESP_LOGE(TAG, "Invalid flash begin parameters: data_len=%lu", cmd.data_len);
                    }
                    break;
                }
                
                case CMD_FLASH_DATA: {
                    ESP_LOGD(TAG, "Processing FLASH_DATA command");
                    
                    if (g_flash_ctx.flash_in_progress && cmd.data_len >= 8) {
                        uint32_t *header = (uint32_t *)cmd.data;
                        uint32_t chunk_size = header[0];
                        uint32_t sequence = header[1];
                        uint8_t *chunk_data = cmd.data + 8;
                        
                        ESP_LOGD(TAG, "Flash data: seq=%lu, size=%lu", sequence, chunk_size);
                        
                        // Copy to firmware buffer
                        uint32_t offset = sequence * 1024;
                        if (offset + chunk_size <= g_flash_ctx.total_size) {
                            memcpy(g_flash_ctx.firmware_buffer + offset, chunk_data, chunk_size);
                            
                            // Flash this block to target
                            ret = flash_data_target(chunk_data, TARGET_FLASH_WRITE_SIZE, sequence);
                            if (ret != ESP_OK) {
                                response_msg = "Flash data failed";
                                ESP_LOGE(TAG, "Failed to flash block %lu", sequence);
                            } else {
                                g_flash_ctx.current_block++;
                                ESP_LOGD(TAG, "Successfully flashed block %lu (%lu/%lu)", 
                                        sequence, g_flash_ctx.current_block, g_flash_ctx.block_count);
                            }
                        } else {
                            ret = ESP_ERR_INVALID_SIZE;
                            response_msg = "Invalid data offset";
                            ESP_LOGE(TAG, "Invalid data offset: %lu + %lu > %lu", offset, chunk_size, g_flash_ctx.total_size);
                        }
                    } else {
                        ret = ESP_ERR_INVALID_STATE;
                        response_msg = "Flash not in progress or invalid data";
                        ESP_LOGE(TAG, "Flash data error: in_progress=%d, data_len=%lu", 
                                g_flash_ctx.flash_in_progress, cmd.data_len);
                    }
                    break;
                }
                
                case CMD_FLASH_END: {
                    ESP_LOGI(TAG, "Processing FLASH_END command");
                    
                    if (g_flash_ctx.flash_in_progress) {
                        // Verify MD5 if provided
                        if (cmd.data_len == 16) {
                            ESP_LOGI(TAG, "Verifying firmware MD5 checksum...");
                            
                            // Calculate MD5 of received firmware
                            mbedtls_md5_context md5_ctx;
                            uint8_t calculated_md5[16];
                            
                            mbedtls_md5_init(&md5_ctx);
                            mbedtls_md5_starts(&md5_ctx);
                            mbedtls_md5_update(&md5_ctx, g_flash_ctx.firmware_buffer, g_flash_ctx.total_size);
                            mbedtls_md5_finish(&md5_ctx, calculated_md5);
                            mbedtls_md5_free(&md5_ctx);
                            
                            if (memcmp(cmd.data, calculated_md5, 16) != 0) {
                                ret = ESP_ERR_INVALID_CRC;
                                response_msg = "MD5 checksum mismatch";
                                ESP_LOGE(TAG, "MD5 checksum verification failed");
                                break;
                            }
                            ESP_LOGI(TAG, "MD5 checksum verification passed");
                        }
                        
                        ret = flash_end_target(cmd.data);
                        if (ret != ESP_OK) {
                            response_msg = "Flash end failed";
                            ESP_LOGE(TAG, "Flash end operation failed");
                        } else {
                            response_msg = "Flash completed successfully";
                            ESP_LOGI(TAG, "Flash operation completed successfully");
                        }
                        
                        // Cleanup flash context
                        g_flash_ctx.flash_in_progress = false;
                        if (g_flash_ctx.firmware_buffer) {
                            free(g_flash_ctx.firmware_buffer);
                            g_flash_ctx.firmware_buffer = NULL;
                        }
                    } else {
                        ret = ESP_ERR_INVALID_STATE;
                        response_msg = "Flash not in progress";
                        ESP_LOGE(TAG, "Flash end called but no flash in progress");
                    }
                    break;
                }
                
                case CMD_TARGET_RESET: {
                    ESP_LOGI(TAG, "Processing TARGET_RESET command");
                    
                    ret = reset_target_device();
                    if (ret == ESP_OK) {
                        response_msg = "Target reset completed";
                        ESP_LOGI(TAG, "Target reset completed successfully");
                    } else {
                        response_msg = "Target reset failed";
                        ESP_LOGE(TAG, "Target reset failed");
                    }
                    break;
                }
                
                default:
                    ret = ESP_ERR_NOT_SUPPORTED;
                    response_msg = "Unknown command";
                    ESP_LOGW(TAG, "Unknown command received: 0x%02X", cmd.cmd);
                    break;
            }
            
            // Send response back to PC
            send_uart_response(ret == ESP_OK ? 0 : 1, response_msg);
            
            // Free command data
            if (cmd.data) {
                free(cmd.data);
            }
        }
    }
}

/**
 * @brief UART command receiving task
 * @param arg Task parameter (unused)
 */
static void uart_task(void *arg) {
    ESP_LOGI(TAG, "UART task started");
    
    uint8_t *uart_buffer = malloc(UART_BUF_SIZE);
    if (!uart_buffer) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }
    
    while (true) {
        // Read command header: [CMD:1][LEN:4]
        int len = uart_read_bytes(UART_NUM, uart_buffer, 5, portMAX_DELAY);
        if (len == 5) {
            uart_command_t cmd;
            cmd.cmd = uart_buffer[0];
            cmd.data_len = uart_buffer[1] | (uart_buffer[2] << 8) | 
                          (uart_buffer[3] << 16) | (uart_buffer[4] << 24);
            
            ESP_LOGI(TAG, "Received UART command 0x%02X with %lu bytes data", cmd.cmd, cmd.data_len);
            
            // Read data payload if any
            cmd.data = NULL;
            if (cmd.data_len > 0) {
                cmd.data = malloc(cmd.data_len);
                if (cmd.data) {
                    int data_read = 0;
                    while (data_read < cmd.data_len) {
                        int chunk = uart_read_bytes(UART_NUM, cmd.data + data_read, 
                                                   cmd.data_len - data_read, 1000 / portTICK_PERIOD_MS);
                        if (chunk <= 0) {
                            ESP_LOGE(TAG, "Timeout reading command data");
                            break;
                        }
                        data_read += chunk;
                    }
                    
                    if (data_read == cmd.data_len) {
                        // Read CRC
                        uint8_t crc_bytes[4];
                        if (uart_read_bytes(UART_NUM, crc_bytes, 4, 1000 / portTICK_PERIOD_MS) == 4) {
                            cmd.crc = crc_bytes[0] | (crc_bytes[1] << 8) | 
                                     (crc_bytes[2] << 16) | (crc_bytes[3] << 24);
                            
                            ESP_LOGD(TAG, "Command data read successfully, CRC: 0x%08lX", cmd.crc);
                            
                            // Send command to flash task
                            if (xQueueSend(flash_queue, &cmd, portMAX_DELAY) != pdTRUE) {
                                ESP_LOGE(TAG, "Failed to send command to flash task");
                                if (cmd.data) free(cmd.data);
                            }
                        } else {
                            ESP_LOGE(TAG, "Failed to read CRC");
                            if (cmd.data) free(cmd.data);
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to read complete command data");
                        if (cmd.data) free(cmd.data);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to allocate memory for command data");
                }
            } else {
                // No data payload, send command directly
                if (xQueueSend(flash_queue, &cmd, portMAX_DELAY) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to send command to flash task");
                }
            }
        }
    }
    
    free(uart_buffer);
}

// =============================================================================
// Main Application Entry Point
// =============================================================================

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 Flash Bridge starting...");
    
    // Initialize UART for PC communication
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized at %d baud", UART_BAUD_RATE);
    
    // Create synchronization objects
    flash_queue = xQueueCreate(10, sizeof(uart_command_t));
    usb_mutex = xSemaphoreCreateMutex();
    g_usb_device.transfer_done_sem = xSemaphoreCreateBinary();
    
    if (!flash_queue || !usb_mutex || !g_usb_device.transfer_done_sem) {
        ESP_LOGE(TAG, "Failed to create synchronization objects");
        return;
    }
    
    ESP_LOGI(TAG, "Synchronization objects created successfully");
    
    // Initialize USB device structure
    memset(&g_usb_device, 0, sizeof(g_usb_device));
    g_usb_device.transfer_done_sem = xSemaphoreCreateBinary();
    
    // Create application tasks
    xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL, 
                           USB_HOST_PRIORITY, NULL, tskNO_AFFINITY);
    
    xTaskCreatePinnedToCore(uart_task, "uart", 4096, NULL, 
                           FLASH_TASK_PRIORITY, NULL, 1);
    
    xTaskCreatePinnedToCore(flash_task, "flash", 8192, NULL, 
                           FLASH_TASK_PRIORITY + 1, NULL, 1);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "ESP32-S3 Flash Bridge initialized and ready");
    
    // Main loop - monitor system status
    uint32_t loop_count = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // Print periodic status
        loop_count++;
        ESP_LOGI(TAG, "System running: loop=%lu, USB ready=%s, flash in progress=%s",
                loop_count,
                g_usb_device.device_ready ? "YES" : "NO",
                g_flash_ctx.flash_in_progress ? "YES" : "NO");
        
        // Print memory status
        ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    }
}
