idf_component_register(SRCS "mindrove_driver.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_netif esp_event)
