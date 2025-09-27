# idf_component_register(SRCS "esp_usb_otg.c" "../usb_handler/usb_handler.c"
#                     INCLUDE_DIRS "../usb_handler/include"
#                     REQUIRES usb esp_driver_gpio)
idf_component_register(SRCS "esp32s3_flash_bridge.c"
                    INCLUDE_DIRS "."
                    REQUIRES usb esp_driver_gpio esp_driver_uart)