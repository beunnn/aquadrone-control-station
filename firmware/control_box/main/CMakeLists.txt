idf_component_register(SRCS "main.c" "joystick.c" "buttons.c" "selection.c" INCLUDE_DIRS "."
                       REQUIRES driver usb esp_timer esp_eth esp_netif esp_event nvs_flash mqtt json mavlink_headers)
