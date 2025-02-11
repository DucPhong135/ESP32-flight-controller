#include "Common-define.h"

#define WIFI_SSID "Xiaomi 12T"   // Replace with your hotspot's SSID
#define WIFI_PASSWORD "12345678" // Replace with your hotspot's password
#define LOCAL_PORT 8080

static const char *RECEIVE_TAG = "receive_task";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(RECEIVE_TAG, "Disconnected from Wi-Fi. Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(RECEIVE_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void ReceiveDataSetup(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register Wi-Fi and IP event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Configure Wi-Fi connection
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Most common for personal hotspots
        },
    };
    ESP_LOGI(RECEIVE_TAG, "Setting WiFi configuration SSID: %s", WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(RECEIVE_TAG, "Waiting for Wi-Fi connection...");
}

void ReceiveDataTask(void *parameter) {
    char rx_buffer[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;  // Use IPv4
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on all interfaces
    local_addr.sin_port = htons(LOCAL_PORT);  // Bind to the specified port

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(RECEIVE_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    // Bind the socket to the local address and port
    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(RECEIVE_TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
    }

    // Set timeout
    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    Control_data_queue = xQueueCreate(30, sizeof(rx_buffer));

    while (1) {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(RECEIVE_TAG, "recvfrom failed: errno %d", errno);
        } 
        // Data received
        else {
            if (strcmp(rx_buffer, "stop") == 0) {
                vTaskDelete(NULL);
            }
            xQueueSend(Control_data_queue, rx_buffer, 0);
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(RECEIVE_TAG, "%s", rx_buffer);
        }
    }
}