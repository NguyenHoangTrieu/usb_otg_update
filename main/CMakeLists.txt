# idf_component_register(SRCS "esp_usb_otg.c" "../usb_handler/usb_handler.c"
#                     INCLUDE_DIRS "../usb_handler/include"
#                     REQUIRES usb esp_driver_gpio)
idf_component_register(SRCS "esp32s3_flash_bridge.c" 
                    "../rbg_led_ws_handler/rbg_handler.c" 
                    "../usb_flash_handler/flash_task.c" 
                    "../usb_flash_handler/uart_task.c" 
                    "../usb_flash_handler/usb_host_task.c"
                    "../usb_flash_handler/rw_task.c"
                    INCLUDE_DIRS "../rbg_led_ws_handler" "../usb_flash_handler/"
                    REQUIRES usb esp_driver_gpio esp_driver_uart mbedtls esp_driver_rmt)