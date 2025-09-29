#ifndef USB_FLASH_HANDLER_H
#define USB_FLASH_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "mbedtls/md5.h"
#include <string.h>
#include "rbg_handler.h"

#define CLIENT_NUM_EVENT_MSG        5
#define CONFIG_APP_QUIT_PIN       5

#define HOST_LIB_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3
#define APP_QUIT_PIN                CONFIG_APP_QUIT_PIN

typedef enum {
    ACTION_OPEN_DEV         = (1 << 0),
    ACTION_GET_DEV_INFO     = (1 << 1),
    ACTION_GET_DEV_DESC     = (1 << 2),
    ACTION_GET_CONFIG_DESC  = (1 << 3),
    ACTION_GET_STR_DESC     = (1 << 4),
    ACTION_CLOSE_DEV        = (1 << 5),
} action_t;

#define DEV_MAX_COUNT           128

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    action_t actions;
} usb_device_t;

typedef struct {
    struct {
        union {
            struct {
                uint8_t unhandled_devices: 1;   /**< Device has unhandled devices */
                uint8_t shutdown: 1;            /**<  */
                uint8_t reserved6: 6;           /**< Reserved */
            };
            uint8_t val;                        /**< Class drivers' flags value */
        } flags;                                /**< Class drivers' flags */
        usb_device_t device[DEV_MAX_COUNT];     /**< Class drivers' static array of devices */
    } mux_protected;                            /**< Mutex protected members. Must be protected by the Class mux_lock when accessed */

    struct {
        usb_host_client_handle_t client_hdl;
        SemaphoreHandle_t mux_lock;         /**< Mutex for protected members */
    } constant;                                 /**< Constant members. Do not change after installation thus do not require a critical section or mutex */
} class_driver_t;

extern esp_err_t usb_cdc_send_data(const uint8_t *data, size_t len);
extern esp_err_t usb_cdc_receive_data(uint8_t *data, size_t max_len, size_t *actual_len);
extern void class_driver_client_deregister(void);

extern void uart_task(void *arg);
extern void flash_task(void *arg);
extern void usb_host_lib_task(void *arg);
extern void class_driver_task(void *arg);
extern void usb_otg_rw_task(void *arg);

#endif // USB_FLASH_HANDLER_H