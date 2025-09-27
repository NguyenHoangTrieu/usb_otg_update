#include "usb_handler.h"

static const char *TAG = "CLASS";
static class_driver_t *s_driver_obj;

/**
 * @brief Client event callback function for handling USB host events
 * 
 * This callback is called by the USB host library when USB device events occur.
 * It handles two main events: device connection and device disconnection.
 * The function is thread-safe and uses mutex protection to update shared data structures.
 * 
 * @param event_msg Pointer to the event message containing event type and data
 * @param arg User argument passed during client registration (class_driver_t pointer)
 */
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        // Save the device address
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);  // Acquire mutex for thread safety
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr = event_msg->new_dev.address;
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_hdl = NULL;
        // Open the device next
        driver_obj->mux_protected.device[event_msg->new_dev.address].actions |= ACTION_OPEN_DEV;
        // Set flag
        driver_obj->mux_protected.flags.unhandled_devices = 1;
        xSemaphoreGive(driver_obj->constant.mux_lock);  // Release mutex
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        // Cancel any other actions and close the device next
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);  // Acquire mutex for thread safety
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
            if (driver_obj->mux_protected.device[i].dev_hdl == event_msg->dev_gone.dev_hdl) {
                driver_obj->mux_protected.device[i].actions = ACTION_CLOSE_DEV;
                // Set flag
                driver_obj->mux_protected.flags.unhandled_devices = 1;
            }
        }
        xSemaphoreGive(driver_obj->constant.mux_lock);  // Release mutex
        break;
    default:
        // Should never occur
        abort();
    }
}

/**
 * @brief Opens a USB device for communication
 * 
 * This function opens a USB device using its address and stores the device handle
 * for future operations. After successfully opening the device, it schedules
 * the next action to get device information.
 * 
 * @param device_obj Pointer to the USB device object containing device address and client handle
 */
static void action_open_dev(usb_device_t *device_obj)
{
    assert(device_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", device_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(device_obj->client_hdl, device_obj->dev_addr, &device_obj->dev_hdl));  // Open USB device using USB Host Library
    // Get the device's information next
    device_obj->actions |= ACTION_GET_DEV_INFO;
}

/**
 * @brief Retrieves and displays USB device information
 * 
 * This function gets detailed information about a USB device including its speed,
 * parent device information, and current configuration value. It also logs
 * the device hierarchy and port connection details.
 * 
 * @param device_obj Pointer to the USB device object with valid device handle
 */
static void action_get_info(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));  // Get device information from USB Host Library
    ESP_LOGI(TAG, "\t%s speed", (char *[]) {
        "Low", "Full", "High"
    }[dev_info.speed]);
    ESP_LOGI(TAG, "\tParent info:");
    if (dev_info.parent.dev_hdl) {
        usb_device_info_t parent_dev_info;
        ESP_ERROR_CHECK(usb_host_device_info(dev_info.parent.dev_hdl, &parent_dev_info));  // Get parent device info if exists
        ESP_LOGI(TAG, "\t\tBus addr: %d", parent_dev_info.dev_addr);
        ESP_LOGI(TAG, "\t\tPort: %d", dev_info.parent.port_num);

    } else {
        ESP_LOGI(TAG, "\t\tPort: ROOT");
    }
    ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    // Get the device descriptor next
    device_obj->actions |= ACTION_GET_DEV_DESC;
}

/**
 * @brief Retrieves and displays the USB device descriptor
 * 
 * This function gets the standard USB device descriptor which contains
 * fundamental information about the device such as vendor ID, product ID,
 * device class, and supported configurations.
 * 
 * @param device_obj Pointer to the USB device object with valid device handle
 */
static void action_get_dev_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(device_obj->dev_hdl, &dev_desc));  // Get device descriptor from USB Host Library
    usb_print_device_descriptor(dev_desc);  // Print device descriptor contents in formatted output
    // Get the device's config descriptor next
    device_obj->actions |= ACTION_GET_CONFIG_DESC;
}

/**
 * @brief Retrieves and displays the USB configuration descriptor
 * 
 * This function gets the configuration descriptor for the currently active
 * configuration. The configuration descriptor contains information about
 * interfaces, endpoints, and other configuration-specific details.
 * 
 * @param device_obj Pointer to the USB device object with valid device handle
 */
static void action_get_config_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl, &config_desc));  // Get active configuration descriptor
    usb_print_config_descriptor(config_desc, NULL);  // Print configuration descriptor with all interfaces and endpoints
    // Get the device's string descriptors next
    device_obj->actions |= ACTION_GET_STR_DESC;
}

/**
 * @brief Retrieves and displays USB string descriptors
 * 
 * This function gets and displays the manufacturer, product, and serial number
 * string descriptors if they are available. These descriptors provide
 * human-readable information about the device.
 * 
 * @param device_obj Pointer to the USB device object with valid device handle
 */
static void action_get_str_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));  // Get device info to access string descriptors
    if (dev_info.str_desc_manufacturer) {
        ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);  // Print manufacturer string
    }
    if (dev_info.str_desc_product) {
        ESP_LOGI(TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);  // Print product string
    }
    if (dev_info.str_desc_serial_num) {
        ESP_LOGI(TAG, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);  // Print serial number string
    }
}

/**
 * @brief Closes a USB device and cleans up resources
 * 
 * This function properly closes a USB device connection and resets the
 * device object's handle and address to indicate it's no longer in use.
 * 
 * @param device_obj Pointer to the USB device object to close
 */
static void action_close_dev(usb_device_t *device_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(device_obj->client_hdl, device_obj->dev_hdl));  // Close USB device using USB Host Library
    device_obj->dev_hdl = NULL;
    device_obj->dev_addr = 0;
}

/**
 * @brief Handles pending actions for a specific USB device
 * 
 * This function processes all pending actions for a USB device in a specific order.
 * It uses a state machine approach where each action can trigger subsequent actions.
 * The function continues processing until no more actions are pending.
 * 
 * @param device_obj Pointer to the USB device object with pending actions
 */
static void class_driver_device_handle(usb_device_t *device_obj)
{
    uint8_t actions = device_obj->actions;
    device_obj->actions = 0;

    while (actions) {
        if (actions & ACTION_OPEN_DEV) {
            action_open_dev(device_obj);  // Open the USB device
        }
        if (actions & ACTION_GET_DEV_INFO) {
            action_get_info(device_obj);  // Get device information
        }
        if (actions & ACTION_GET_DEV_DESC) {
            action_get_dev_desc(device_obj);  // Get device descriptor
        }
        if (actions & ACTION_GET_CONFIG_DESC) {
            action_get_config_desc(device_obj);  // Get configuration descriptor
        }
        if (actions & ACTION_GET_STR_DESC) {
            action_get_str_desc(device_obj);  // Get string descriptors
        }
        if (actions & ACTION_CLOSE_DEV) {
            action_close_dev(device_obj);  // Close the device
        }

        actions = device_obj->actions;
        device_obj->actions = 0;
    }
}

/**
 * @brief Main task function for the USB class driver
 * 
 * This is the main task that runs the USB class driver. It initializes the driver,
 * registers with the USB host library, and enters the main event loop.
 * The task handles two main states:
 * 1. Processing pending device actions when devices need attention
 * 2. Waiting for and handling USB host client events
 * 
 * The task runs until shutdown is requested, then properly deregisters
 * the client and cleans up resources.
 * 
 * @param arg Task parameter (unused in this implementation)
 */
void class_driver_task(void *arg)
{
    class_driver_t driver_obj = {0};
    usb_host_client_handle_t class_driver_client_hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");

    SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();  // Create mutex for thread synchronization
    if (mux_lock == NULL) {
        ESP_LOGE(TAG, "Unable to create class driver mutex");
        vTaskSuspend(NULL);  // Suspend task if mutex creation fails
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *) &driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &class_driver_client_hdl));  // Register client with USB Host Library

    driver_obj.constant.mux_lock = mux_lock;
    driver_obj.constant.client_hdl = class_driver_client_hdl;

    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
        driver_obj.mux_protected.device[i].client_hdl = class_driver_client_hdl;
    }

    s_driver_obj = &driver_obj;

    while (1) {
        // Driver has unhandled devices, handle all devices first
        if (driver_obj.mux_protected.flags.unhandled_devices) {
            xSemaphoreTake(driver_obj.constant.mux_lock, portMAX_DELAY);  // Acquire mutex for thread safety
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
                if (driver_obj.mux_protected.device[i].actions) {
                    class_driver_device_handle(&driver_obj.mux_protected.device[i]);  // Process device actions
                }
            }
            driver_obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(driver_obj.constant.mux_lock);  // Release mutex
        } else {
            // Driver is active, handle client events
            if (driver_obj.mux_protected.flags.shutdown == 0) {
                usb_host_client_handle_events(class_driver_client_hdl, portMAX_DELAY);  // Wait for and handle USB client events
            } else {
                // Shutdown the driver
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(class_driver_client_hdl));  // Deregister client from USB Host Library
    if (mux_lock != NULL) {
        vSemaphoreDelete(mux_lock);  // Delete mutex to free resources
    }
    vTaskSuspend(NULL);  // Suspend task after cleanup
}

/**
 * @brief Initiates class driver shutdown and cleanup
 * 
 * This function triggers the shutdown process for the class driver.
 * It marks all open devices for closure, sets the shutdown flag,
 * and unblocks the main driver task to allow it to complete the
 * shutdown process gracefully.
 * 
 * This function is typically called from outside the driver task
 * when the application needs to shut down the USB functionality.
 */
void class_driver_client_deregister(void)
{
    // Mark all opened devices
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);  // Acquire mutex for thread safety
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL) {
            // Mark device to close
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            // Set flag
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);  // Release mutex

    // Unblock, exit the loop and proceed to deregister client
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));  // Unblock client to allow graceful shutdown
}
