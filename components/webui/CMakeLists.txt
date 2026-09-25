idf_component_register(SRCS "webui.c" "wifi_ap.c"
                       INCLUDE_DIRS "include"
                       REQUIRES core esp_http_server esp_wifi esp_netif
                                esp_event nvs_flash
                       EMBED_FILES "www/index.html" "www/app.js" "www/style.css")
