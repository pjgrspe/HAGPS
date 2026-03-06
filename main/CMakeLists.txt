idf_component_register(
    SRCS
        "main.c"
        "gnss_uart.c"
        "nmea_parser.c"
        "mqtt_comm.c"
        "wifi_comm.c"
        "wifi_config.c"
        "role_config.c"
        "command_handler.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        mqtt esp_wifi nvs_flash esp_netif esp_event driver
)
