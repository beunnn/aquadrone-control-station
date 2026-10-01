idf_component_register(SRCS "main.c" "joystick.c" "buttons.c" INCLUDE_DIRS "."
                       REQUIRES driver usb esp_timer esp_eth esp_netif esp_event nvs_flash mavlink_headers)
