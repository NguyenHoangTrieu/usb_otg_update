#include "usb_flash_handler.h"

static const char *TAG = "USB_OTG_RW";

static void transfer_cb(usb_transfer_t *transfer) {
    usb_host_transfer_free(transfer);
}

/** Claims the interface for the given device.
 * This is necessary before performing any data transfers.
 * @param dev Pointer to usb_device_t struct with valid dev_hdl and
 * interface_num
 */
void claim_interface(usb_device_t *device_obj) {
  ESP_ERROR_CHECK(usb_host_interface_claim(device_obj->client_hdl,
                                           device_obj->dev_hdl,
                                           device_obj->interface_num, 0));
  ESP_LOGI(TAG, "Interface %d claimed for device addr %d",
           device_obj->interface_num, device_obj->dev_addr);
}

typedef struct {
    SemaphoreHandle_t completion_sem;
    esp_err_t result;
    size_t actual_len;
    uint8_t *user_buffer;
} transfer_context_t;

static void receive_transfer_cb(usb_transfer_t *transfer)
{
    transfer_context_t *ctx = (transfer_context_t *)transfer->context;
    
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        // Copy data to user buffer
        memcpy(ctx->user_buffer, transfer->data_buffer, transfer->actual_num_bytes);
        ctx->actual_len = transfer->actual_num_bytes;
        ctx->result = ESP_OK;
        
        ESP_LOGI("USBOTG", "Received %d bytes from endpoint 0x%02X", 
                 transfer->actual_num_bytes, transfer->bEndpointAddress);
    } else if (transfer->status == USB_TRANSFER_STATUS_NO_DEVICE) {
        ESP_LOGW("USBOTG", "Device disconnected during transfer");
        ctx->result = ESP_ERR_NOT_FOUND;
        ctx->actual_len = 0;
    } else {
        ESP_LOGW("USBOTG", "Transfer failed with status: %d", transfer->status);
        ctx->result = ESP_FAIL;
        ctx->actual_len = 0;
    }
    
    // Signal completion
    xSemaphoreGive(ctx->completion_sem);
}

/** Parses endpoints for CDC/Data class. Call in action_get_config_desc or after
 * enumeration. Caches endpoint addresses in usb_device_t struct for later use.
 * @param dev Pointer to usb_device_t struct with valid dev_hdl
 */
void parse_and_cache_endpoints(usb_device_t *dev) 
{
    const usb_config_desc_t *config_desc = NULL;
    dev->ep_out_addr = 0x00;
    dev->ep_in_addr = 0x00;
    dev->interface_num = 0;
    
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(dev->dev_hdl, &config_desc));
    
    uint8_t found_cdc_data_interface = 0;
    uint8_t target_interface = 0;
    
    // Phase 1: Scan all interfaces to find CDC Data class (0x0A) or suitable interface
    int offset = 0;
    while (offset < config_desc->wTotalLength) {
        const usb_standard_desc_t *desc = 
            (const usb_standard_desc_t *)(((const uint8_t *)config_desc) + offset);
        
        if (desc->bDescriptorType == USB_DESC_TYPE_INTERFACE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc;
            
            ESP_LOGI(TAG, "Interface found: num=%u, class=0x%02X, endpoints=%u",
                     intf->bInterfaceNumber, intf->bInterfaceClass, intf->bNumEndpoints);
            
            // Prioritize CDC Data class (0x0A) for CDC ACM devices
            if (intf->bInterfaceClass == 0x0A && intf->bNumEndpoints >= 2) {
                target_interface = intf->bInterfaceNumber;
                found_cdc_data_interface = 1;
                ESP_LOGI(TAG, "Found CDC Data Interface %u (class 0x0A)", target_interface);
                break;  // Found CDC Data interface, use this one
            }
            
            // Fallback: Use first interface with >= 2 endpoints (vendor serial like CH340)
            if (!found_cdc_data_interface && intf->bNumEndpoints >= 2) {
                target_interface = intf->bInterfaceNumber;
                ESP_LOGI(TAG, "Using vendor serial interface %u (class 0x%02X)", 
                         target_interface, intf->bInterfaceClass);
            }
        }
        
        offset += desc->bLength;
    }
    
    // Phase 2: Parse endpoints from the target interface
    offset = 0;
    uint8_t parsing_target_interface = 0;
    
    while (offset < config_desc->wTotalLength) {
        const usb_standard_desc_t *desc = 
            (const usb_standard_desc_t *)(((const uint8_t *)config_desc) + offset);
        
        if (desc->bDescriptorType == USB_DESC_TYPE_INTERFACE) {
            const usb_intf_desc_t *intf = (const usb_intf_desc_t *)desc;
            
            if (intf->bInterfaceNumber == target_interface) {
                dev->interface_num = target_interface;
                parsing_target_interface = 1;
                ESP_LOGI(TAG, "Parsing endpoints for interface %u", target_interface);
            } else {
                parsing_target_interface = 0;  // Moved to different interface
            }
        }
        else if (desc->bDescriptorType == USB_DESC_TYPE_ENDPOINT && parsing_target_interface) {
            const usb_ep_desc_t *ep = (const usb_ep_desc_t *)desc;
            uint8_t ep_addr = ep->bEndpointAddress;
            uint8_t ep_type = ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK;
            
            // Only care about BULK endpoints for data transfer
            if (ep_type == USB_BM_ATTRIBUTES_XFER_BULK) {
                if (ep_addr & 0x80) {
                    dev->ep_in_addr = ep_addr;
                    ESP_LOGI(TAG, "-- Found BULK IN: addr=0x%02X, MPS=%u", 
                             ep_addr, ep->wMaxPacketSize);
                } else {
                    dev->ep_out_addr = ep_addr;
                    ESP_LOGI(TAG, "-- Found BULK OUT: addr=0x%02X, MPS=%u", 
                             ep_addr, ep->wMaxPacketSize);
                }
            }
            
            // If we have both endpoints, we're done
            if (dev->ep_in_addr != 0x00 && dev->ep_out_addr != 0x00) {
                break;
            }
        }
        
        offset += desc->bLength;
    }
    
    // Log final result
    ESP_LOGI(TAG, "Parsed endpoints: OUT=0x%02X, IN=0x%02X (Interface=%u)",
             dev->ep_out_addr, dev->ep_in_addr, dev->interface_num);
    
    if (dev->ep_out_addr == 0x00 || dev->ep_in_addr == 0x00) {
        ESP_LOGE(TAG, "USB serial parsing failed. No valid BULK endpoints assigned!");
    } else {
        ESP_LOGI(TAG, "Successfully parsed %s device", 
                 found_cdc_data_interface ? "CDC ACM" : "vendor serial");
    }
}
/**
 * @brief Send data to USB device via OUT endpoint
 *
 * @param[in] dev          USB device (usb_device_t)
 * @param[in] data         Data buffer to send
 * @param[in] len          Number of bytes in data buffer
 * @param[in] timeout_ms   Timeout in milliseconds
 * @return esp_err_t   ESP_OK if successful, error code otherwise
 */
esp_err_t usb_cdc_send_data(usb_device_t *dev, const uint8_t *data, size_t len,
                            int timeout_ms) {
  esp_err_t err;
  usb_transfer_t *transfer = NULL;

  if (!dev || dev->dev_hdl == NULL) {
    ESP_LOGE("USBOTG", "Invalid device handle");
    return ESP_ERR_INVALID_ARG;
  }

  err = usb_host_transfer_alloc(len, 0, &transfer);
  if (err != ESP_OK) {
    ESP_LOGE("USBOTG", "Failed to allocate transfer struct");
    return err;
  }

  transfer->device_handle = dev->dev_hdl;
  transfer->num_bytes = len;
  transfer->bEndpointAddress =
      dev->ep_out_addr; // endpoint_out needs to be initialized from descriptor
  transfer->timeout_ms = timeout_ms;
  transfer->callback = transfer_cb; // transfer_cb;
  memcpy(transfer->data_buffer, data, len);
  err = usb_host_transfer_submit(transfer);
  if (err == ESP_OK) {
    ESP_LOGI("USBOTG", "Sent %d bytes to device: endpoint 0x%02X", (int)len,
             dev->ep_out_addr);
  } else {
    ESP_LOGE("USBOTG", "USB Send failed: %d", err);
  }
  return err;
}

/**
 * @brief Receive data from USB device via IN endpoint
 *
 * @param[in] dev          USB device (usb_device_t)
 * @param[out] data        Buffer to store received data
 * @param[in] max_len      Maximum number of bytes to receive (size of data
 * buffer)
 * @param[out] actual_len  Pointer to store actual number of bytes received
 *
 * @return esp_err_t         ESP_OK if successful, error code otherwise
 */
esp_err_t usb_cdc_receive_data(usb_device_t *dev, uint8_t *data, size_t max_len,
                               size_t *actual_len) {
    esp_err_t err;
    usb_transfer_t *transfer = NULL;
    
    if (!dev || dev->dev_hdl == NULL) {
        ESP_LOGE("USBOTG", "Invalid device handle");
        *actual_len = 0;
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create context for this transfer
    transfer_context_t ctx = {
        .completion_sem = xSemaphoreCreateBinary(),
        .result = ESP_FAIL,
        .actual_len = 0,
        .user_buffer = data
    };
    
    if (ctx.completion_sem == NULL) {
        ESP_LOGE("USBOTG", "Failed to create semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate transfer
    err = usb_host_transfer_alloc(max_len, 0, &transfer);
    if (err != ESP_OK) {
        ESP_LOGE("USBOTG", "Failed to allocate transfer struct");
        vSemaphoreDelete(ctx.completion_sem);
        *actual_len = 0;
        return err;
    }
    
    // Setup transfer
    transfer->device_handle = dev->dev_hdl;
    transfer->num_bytes = max_len;
    transfer->bEndpointAddress = dev->ep_in_addr;
    transfer->callback = receive_transfer_cb;
    transfer->context = &ctx;
    
    // Submit transfer
    err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
        ESP_LOGE("USBOTG", "USB transfer submit failed: %s", esp_err_to_name(err));
        usb_host_transfer_free(transfer);
        vSemaphoreDelete(ctx.completion_sem);
        *actual_len = 0;
        return err;
    }
    
    // WAIT for transfer completion
    if (xSemaphoreTake(ctx.completion_sem, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Transfer completed
        *actual_len = ctx.actual_len;
        err = ctx.result;
    } else {
        // Timeout
        ESP_LOGW("USBOTG", "Transfer timeout");
        *actual_len = 0;
        err = ESP_ERR_TIMEOUT;
        
        // Cancel the transfer if possible
        // Note: ESP-IDF doesn't have transfer cancel, so we just wait
    }
    
    // Cleanup
    usb_host_transfer_free(transfer);
    vSemaphoreDelete(ctx.completion_sem);
    
    return err;
}

void ch340_set_baudrate(usb_device_t *dev) {
    uint32_t divisor = 1532620800UL / 115200UL;
    if (divisor > 0) divisor--;
    uint16_t value = divisor & 0xFFFF;
    uint16_t index = ((divisor >> 8) & 0xFF) | 0x0080;

    // First setup packet
    usb_transfer_t *ctrl1 = NULL;
    ESP_ERROR_CHECK(usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &ctrl1));
    ctrl1->device_handle = dev->dev_hdl;
    ctrl1->bEndpointAddress = 0;
    ctrl1->callback = transfer_cb;
    ctrl1->context = NULL;
    ctrl1->num_bytes = sizeof(usb_setup_packet_t);

    usb_setup_packet_t setup1 = {
        .bmRequestType = 0x40,
        .bRequest      = 0x9A,
        .wValue        = 0x1312,
        .wIndex        = value,
        .wLength       = 0,
    };
    memcpy(ctrl1->data_buffer, &setup1, sizeof(setup1));
    ESP_ERROR_CHECK(usb_host_transfer_submit_control(dev->client_hdl, ctrl1));
    vTaskDelay(pdMS_TO_TICKS(10));
    // Second setup packet
    usb_transfer_t *ctrl2 = NULL;
    ESP_ERROR_CHECK(usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &ctrl2));
    ctrl2->device_handle = dev->dev_hdl;
    ctrl2->bEndpointAddress = 0;
    ctrl2->callback = transfer_cb;
    ctrl2->context = NULL;
    ctrl2->num_bytes = sizeof(usb_setup_packet_t);

    usb_setup_packet_t setup2 = {
        .bmRequestType = 0x40,
        .bRequest      = 0x9A,
        .wValue        = 0x0F2C,
        .wIndex        = index,
        .wLength       = 0,
    };
    memcpy(ctrl2->data_buffer, &setup2, sizeof(setup2));
    ESP_ERROR_CHECK(usb_host_transfer_submit_control(dev->client_hdl, ctrl2));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI("CH340", "Configured baudrate to 115200 for CH340.");
}

// =============================================================================
// Task Implementations
// =============================================================================

/**
 * @brief Example FreeRTOS task that performs USB OTG read and write operations
 *
 * This task sends a buffer of data to the USB device,
 * then waits to receive a response back in a loop.
 */
void usb_otg_rw_task(void *arg) {
  uint8_t tx_data[] = {'N', 'A', 'T', 'E', ' ', 'H', 'I', 'G', 'G', 'E', 'R', '\n'}; // Example buffer to send
  uint8_t rx_data[64];                          // Buffer for receiving data
  size_t actual_len = 0;
  uint8_t configured = 0;

  while (true) {
    usb_device_t *dev = NULL;
    // Protect access to driver handle via mutex
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
      if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL) {
        dev = &s_driver_obj->mux_protected.device[i]; // Get the first opened device handle
        break;
      }
    }
    xSemaphoreGive(s_driver_obj->constant.mux_lock);

    if (dev != NULL) {
      // If device found and not yet configured, set baudrate for CH340
      if (configured == 0) {
        ch340_set_baudrate(dev);
        configured = 1; // Only configure once
        ESP_LOGI("USB_OTG_RW", "CH340 baudrate set to 19200");
      }
      // New: Verify device is still valid
      usb_device_info_t dev_info;
      esp_err_t err = usb_host_device_info(dev->dev_hdl, &dev_info);
      if (err != ESP_OK) {
        ESP_LOGE("USB_OTG_RW",
                 "Device invalid (err: %d) - likely disconnected. Skipping.",
                 err);
        led_show_red(); // Indicate error
        continue;       // Skip transfers
      }

      // Proceed with send/receive as before
      led_show_blue(); // Indicate busy
      esp_err_t send_ret =
          usb_cdc_send_data(dev, tx_data, sizeof(tx_data), 100);
      if (send_ret == ESP_OK) {
        ESP_LOGI("USB_OTG_RW", "Sent %d bytes.", (int)sizeof(tx_data));
      } else {
        ESP_LOGE("USB_OTG_RW", "Send error: %d", send_ret);
        led_show_red(); // Indicate error
      }

      esp_err_t recv_ret =
          usb_cdc_receive_data(dev, rx_data, sizeof(rx_data), &actual_len);
      if (recv_ret == ESP_OK && actual_len > 0) {
        ESP_LOGI("USB_OTG_RW", "Received %d bytes:", (int)actual_len);
        for (size_t i = 0; i < actual_len; ++i) {
          printf("%02X ", rx_data[i]);
        }
        printf("\n");
      } else {
        ESP_LOGW("USB_OTG_RW", "No data received or error: %d", recv_ret);
      }
      led_show_green(); // Indicate done
    } else {
      ESP_LOGW("USB_OTG_RW", "No USB device currently opened. Task idle.");
      led_toggle_white(); // Indicate idle
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}