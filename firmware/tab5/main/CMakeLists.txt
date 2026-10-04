idf_component_register(SRCS "main.c" "wifi.c" "link.c" "selection.c" "ui.c" INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_netif esp_event esp_timer esp_wifi lwip mqtt json mavlink_headers)
