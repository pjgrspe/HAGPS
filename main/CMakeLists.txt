idf_component_register(SRCS "mqtt_comm.c" "wifi_comm.c" "main.c" "uart_gnss.c" "espnow_comm.c" "rover_espnow_receiver.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi nvs_flash esp_netif esp_event driver)
