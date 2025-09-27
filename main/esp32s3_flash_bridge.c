/*
 * ESP32-S3 USB Host Flash Bridge
 * Receives firmware from PC via UART and flashes target ESP32 WROOM via USB Host
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
#include "usb_host_cdc_acm.h"
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

typedef struct {
    uint8_t cmd;
    uint32_t data_len;
    uint8_t *data;
    uint32_t crc;
} uart_command_t;

typedef struct {
    uint32_t total_size;
    uint32_t block_count;
    uint32_t current_block;
    uint32_t target_address;
    uint8_t *firmware_buffer;
    bool flash_in_progress;
} flash_context_t;

static flash_context_t g_flash_ctx = {0};
static cdc_acm_dev_hdl_t cdc_dev_handle = NULL;
static QueueHandle_t flash_queue = NULL;
static SemaphoreHandle_t usb_mutex = NULL;

// ESP32 ROM Bootloader SLIP protocol
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

static size_t slip_encode(const uint8_t *data, size_t len, uint8_t *encoded) {
    size_t encoded_len = 0;
    encoded[encoded_len++] = 0xC0; // Start delimiter
    
    for (size_t i = 0; i < len; i++) {
        slip_encode_byte(data[i], encoded, &encoded_len);
    }
    
    encoded[encoded_len++] = 0xC0; // End delimiter
    return encoded_len;
}

static esp_err_t send_esp32_command(uint8_t cmd, const uint8_t *data, size_t data_len, 
                                   uint32_t checksum) {
    if (!cdc_dev_handle) {
        ESP_LOGE(TAG, "CDC device not connected");
        return ESP_FAIL;
    }
    
    // Build ESP32 command packet
    uint8_t packet[1024];
    size_t packet_len = 0;
    
    packet[packet_len++] = 0x00; // Direction (request)
    packet[packet_len++] = cmd;   // Command
    packet[packet_len++] = data_len & 0xFF;        // Size low
    packet[packet_len++] = (data_len >> 8) & 0xFF; // Size high
    packet[packet_len++] = checksum & 0xFF;        // Checksum
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
    
    // Send via USB CDC-ACM
    xSemaphoreTake(usb_mutex, portMAX_DELAY);
    esp_err_t ret = cdc_acm_host_data_tx_blocking(cdc_dev_handle, encoded, encoded_len, 1000);
    xSemaphoreGive(usb_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X: %s", cmd, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Sent command 0x%02X with %d bytes", cmd, data_len);
    return ESP_OK;
}

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

static esp_err_t flash_begin_target(uint32_t size, uint32_t blocks, uint32_t address) {
    ESP_LOGI(TAG, "Starting flash operation: size=%lu, blocks=%lu, addr=0x%lx", 
             size, blocks, address);
    
    uint8_t flash_begin_data[16];
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

static esp_err_t flash_data_target(const uint8_t *data, uint32_t size, uint32_t sequence) {
    ESP_LOGD(TAG, "Flashing block %lu (%lu bytes)", sequence, size);
    
    // Prepare flash data packet: [SIZE:4][SEQ:4][RESERVED:8][DATA]
    uint8_t flash_packet[TARGET_FLASH_WRITE_SIZE + 16];
    
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

static void usb_host_task(void *arg) {
    ESP_LOGI(TAG, "Starting USB Host task");
    
    // Install USB Host library
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host library installed");
    
    // Initialize CDC-ACM Host driver
    const cdc_acm_host_driver_config_t cdc_acm_config = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = USB_HOST_PRIORITY,
        .xCoreID = tskNO_AFFINITY,
    };
    
    ESP_ERROR_CHECK(cdc_acm_host_install(&cdc_acm_config));
    ESP_LOGI(TAG, "CDC-ACM Host driver installed");
    
    // Handle USB events
    while (true) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "No more USB clients");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed");
        }
    }
}

static void cdc_acm_host_device_event_cb(const cdc_acm_host_dev_event_data_t *event, void *user_ctx) {
    switch (event->type) {
        case CDC_ACM_HOST_DEVICE_CONNECTED:
            ESP_LOGI(TAG, "CDC-ACM device connected");
            cdc_dev_handle = event->data.connected.dev_hdl;
            
            // Configure line coding for target ESP32
            const cdc_acm_line_coding_t line_coding = {
                .dwDTERate = ESP32_BOOTLOADER_BAUD,
                .bCharFormat = 0,
                .bParityType = 0,
                .bDataBits = 8,
            };
            
            cdc_acm_host_line_coding_set(cdc_dev_handle, &line_coding);
            
            // Set DTR and RTS for ESP32 reset/boot control
            cdc_acm_host_set_control_line_state(cdc_dev_handle, true, false);
            break;
            
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "CDC-ACM device disconnected");
            cdc_dev_handle = NULL;
            break;
            
        case CDC_ACM_HOST_ERROR:
            ESP_LOGE(TAG, "CDC-ACM error");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown CDC-ACM event: %d", event->type);
            break;
    }
}

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

static void flash_task(void *arg) {
    ESP_LOGI(TAG, "Flash task started");
    
    uart_command_t cmd;
    
    while (true) {
        if (xQueueReceive(flash_queue, &cmd, portMAX_DELAY)) {
            esp_err_t ret = ESP_OK;
            const char *response_msg = "OK";
            
            switch (cmd.cmd) {
                case CMD_FLASH_BEGIN: {
                    if (cmd.data_len >= 12) {
                        uint32_t *params = (uint32_t *)cmd.data;
                        g_flash_ctx.total_size = params[0];
                        g_flash_ctx.block_count = params[1];  
                        g_flash_ctx.target_address = params[2];
                        g_flash_ctx.current_block = 0;
                        g_flash_ctx.flash_in_progress = true;
                        
                        // Allocate firmware buffer
                        if (g_flash_ctx.firmware_buffer) {
                            free(g_flash_ctx.firmware_buffer);
                        }
                        g_flash_ctx.firmware_buffer = malloc(g_flash_ctx.total_size);
                        
                        if (!g_flash_ctx.firmware_buffer) {
                            ret = ESP_ERR_NO_MEM;
                            response_msg = "Memory allocation failed";
                        } else {
                            // Sync with target and begin flash
                            ret = sync_target_esp32();
                            if (ret == ESP_OK) {
                                ret = flash_begin_target(g_flash_ctx.total_size, 
                                                       g_flash_ctx.block_count,
                                                       g_flash_ctx.target_address);
                            }
                            if (ret != ESP_OK) {
                                response_msg = "Flash begin failed";
                            }
                        }
                    } else {
                        ret = ESP_ERR_INVALID_ARG;
                        response_msg = "Invalid flash begin parameters";
                    }
                    break;
                }
                
                case CMD_FLASH_DATA: {
                    if (g_flash_ctx.flash_in_progress && cmd.data_len >= 8) {
                        uint32_t *header = (uint32_t *)cmd.data;
                        uint32_t chunk_size = header[0];
                        uint32_t sequence = header[1];
                        uint8_t *chunk_data = cmd.data + 8;
                        
                        // Copy to firmware buffer
                        uint32_t offset = sequence * 1024;
                        if (offset + chunk_size <= g_flash_ctx.total_size) {
                            memcpy(g_flash_ctx.firmware_buffer + offset, chunk_data, chunk_size);
                            
                            // Flash this block to target
                            ret = flash_data_target(chunk_data, TARGET_FLASH_WRITE_SIZE, sequence);
                            if (ret != ESP_OK) {
                                response_msg = "Flash data failed";
                            } else {
                                g_flash_ctx.current_block++;
                            }
                        } else {
                            ret = ESP_ERR_INVALID_SIZE;
                            response_msg = "Invalid data offset";
                        }
                    } else {
                        ret = ESP_ERR_INVALID_STATE;
                        response_msg = "Flash not in progress or invalid data";
                    }
                    break;
                }
                
                case CMD_FLASH_END: {
                    if (g_flash_ctx.flash_in_progress) {
                        // Verify MD5 if provided
                        if (cmd.data_len == 16) {
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
                                break;
                            }
                        }
                        
                        ret = flash_end_target(cmd.data);
                        if (ret != ESP_OK) {
                            response_msg = "Flash end failed";
                        } else {
                            response_msg = "Flash completed successfully";
                        }
                        
                        g_flash_ctx.flash_in_progress = false;
                        if (g_flash_ctx.firmware_buffer) {
                            free(g_flash_ctx.firmware_buffer);
                            g_flash_ctx.firmware_buffer = NULL;
                        }
                    } else {
                        ret = ESP_ERR_INVALID_STATE;
                        response_msg = "Flash not in progress";
                    }
                    break;
                }
                
                case CMD_TARGET_RESET: {
                    // Reset target ESP32 by toggling DTR/RTS
                    if (cdc_dev_handle) {
                        cdc_acm_host_set_control_line_state(cdc_dev_handle, false, true); // RTS=1, DTR=0 (reset)
                        vTaskDelay(pdMS_TO_TICKS(100));
                        cdc_acm_host_set_control_line_state(cdc_dev_handle, true, false);  // RTS=0, DTR=1 (run)
                        response_msg = "Target reset";
                    } else {
                        ret = ESP_ERR_INVALID_STATE;
                        response_msg = "Target not connected";
                    }
                    break;
                }
                
                default:
                    ret = ESP_ERR_NOT_SUPPORTED;
                    response_msg = "Unknown command";
                    break;
            }
            
            // Send response
            send_uart_response(ret == ESP_OK ? 0 : 1, response_msg);
            
            // Free command data
            if (cmd.data) {
                free(cmd.data);
            }
        }
    }
}

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
            
            ESP_LOGI(TAG, "Received command 0x%02X with %lu bytes", cmd.cmd, cmd.data_len);
            
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

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-S3 Flash Bridge starting...");
    
    // Initialize UART
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
    
    // Create synchronization objects
    flash_queue = xQueueCreate(10, sizeof(uart_command_t));
    usb_mutex = xSemaphoreCreateMutex();
    
    if (!flash_queue || !usb_mutex) {
        ESP_LOGE(TAG, "Failed to create synchronization objects");
        return;
    }
    
    // Register CDC-ACM Host device callback
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000,
        .out_buffer_size = 512,
        .user_arg = NULL,
        .event_cb = cdc_acm_host_device_event_cb,
    };
    ESP_ERROR_CHECK(cdc_acm_host_install_device_event_callback(&dev_config));
    
    // Create tasks
    xTaskCreatePinnedToCore(usb_host_task, "usb_host", 4096, NULL, 
                           USB_HOST_PRIORITY, NULL, tskNO_AFFINITY);
    
    xTaskCreatePinnedToCore(uart_task, "uart", 4096, NULL, 
                           FLASH_TASK_PRIORITY, NULL, 1);
    
    xTaskCreatePinnedToCore(flash_task, "flash", 8192, NULL, 
                           FLASH_TASK_PRIORITY + 1, NULL, 1);
    
    ESP_LOGI(TAG, "ESP32-S3 Flash Bridge initialized");
    
    // Main loop - could add status monitoring here
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
