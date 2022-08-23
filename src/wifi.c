#include <sys/socket.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <mdns.h>
#include <netdb.h>
#include <driver/spi_slave.h>
#include <errno.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>

#include "beacon.h"

#define DEFAULT_SSID "NCSpeople"
#define DEFAULT_PASSWORD "peopleNCS"
#define TCP_PORT 55671

uint8_t user_ssid[32];
uint8_t user_password[64];

int tcp_socket = -1;
bool wifi_connected = false;

wifi_config_t wifi_config = {
    .sta = {
        .ssid = DEFAULT_SSID,
        .password = DEFAULT_PASSWORD,
        .scan_method = WIFI_ALL_CHANNEL_SCAN,
        .bssid_set = 0,
        .channel = 0,
        .listen_interval = 0,
        .pmf_cfg = {
            .required = 0
        },
        .rm_enabled = 1,
        .btm_enabled = 1,
    },
};

// returns 1 if error
bool yielding_read(int client, uint8_t *buf, size_t len) {
    size_t received = 0;
    while (received != len) {
        int bytes_received = recv(client, buf + received, len - received, MSG_DONTWAIT);

        if (bytes_received < 0 ) {
            if (errno == ERR_WOULDBLOCK) {
                vTaskDelay(10);
                continue;
            }
            if (errno == ENOTCONN) {
                return 1;
                break;
            }

            vTaskDelay(10);
            continue;
        }
        if (bytes_received > 0) {
            received += bytes_received;
            printf("got %d bytes\n", bytes_received);
        }
    }
    return 0;
}

void tcp_server_task(void *user) {
    struct sockaddr_in remote_addr;
	unsigned int socklen;
	socklen = sizeof(remote_addr);
    uint8_t rx_buf[512];
    uint8_t tx_buf[32];

    while (1) {
        vTaskDelay(10);
        if (!wifi_connected) {
            printf("no wifi\n");
            vTaskDelay(100);
            continue;
        }
        int client_socket = -1;
        client_socket = accept(tcp_socket,(struct sockaddr *)&remote_addr, &socklen);
        
        if (client_socket < 0) {
            printf("no client\n");
            vTaskDelay(100);
            continue;
        }
        fcntl(tcp_socket, F_SETFL, O_NONBLOCK);
        // handle client loop
        while (1) {
            uint8_t id;
            if (yielding_read(client_socket, &id, 1)) {
                break;
            }

            if (id == 2) { // handshake
                uint8_t response = 42;
                send(client_socket, &response, 1, 0);
            }

            if (id == 3) { // update wifi config
                if (yielding_read(client_socket, rx_buf, 96)) {
                    break;
                }
                memcpy(user_ssid, rx_buf, 32);
                memcpy(user_password, rx_buf + 32, 64);
                nvs_handle_t nvs;
                nvs_open("storage", NVS_READWRITE, &nvs);
                nvs_set_blob(nvs, "user_ssid", user_ssid, sizeof(user_ssid));
                nvs_set_blob(nvs, "user_password", user_password, sizeof(user_password));
                uint8_t ack = 1;
                send(client_socket, &ack, 1, 0);
            }

            if (id == 4) { // set node id
                if (yielding_read(client_socket, &node_id, 1)) {
                    break;
                }
                nvs_handle_t nvs;
                ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
                ESP_ERROR_CHECK(nvs_set_blob(nvs, "node_id", &node_id, sizeof(node_id)));
                printf("set node id %d\n", node_id);
                uint8_t ack = 1;
                send(client_socket, &ack, 1, 0);
            }

            if (id == 32) { // set mode
                if (yielding_read(client_socket, &mode, 1)) {
                    break;
                }
                uint8_t ack = 1;
                send(client_socket, &ack, 1, 0);
            }

            if (id == 33) { // get position
                tx_buf[0] = 34;
                memcpy(tx_buf + 1, &pos_x, 4);
                memcpy(tx_buf + 5, &pos_y, 4);
                memcpy(tx_buf + 9, &pos_z, 4);
                send(client_socket, tx_buf, 13, 0);
            }

            if (id == 42) { // download firmware
                uint64_t size; // little endian
                uint64_t written = 0;
                if (yielding_read(client_socket, &size, 8)) {
                    break;
                }
                printf("starting firmware update with size%lld\n", size);
                if (size == 0) { // confirming prior update
                    const esp_partition_t *part = esp_ota_get_running_partition();
                    esp_ota_img_states_t state;
                    esp_ota_get_state_partition(part, &state);
                    if (state == ESP_OTA_IMG_NEW || state == ESP_OTA_IMG_PENDING_VERIFY) {
                        int e = esp_ota_mark_app_valid_cancel_rollback();
                        if (e == 0) {
                            uint8_t ack = 1;
                            send(client_socket, &ack, 1, 0);
                            printf("confirmed update\n");
                        } else {
                            uint8_t nak = 1;
                            send(client_socket, &nak, 1, 0);
                            printf("Something went wrong when confirming\n");
                        }
                    } else {
                        uint8_t nak = 0;
                        send(client_socket, &nak, 1, 0);
                        printf("update confirmation not needed %d\n", state);
                    }
                } else {
                    esp_partition_t *ota_partition;
                    esp_ota_handle_t ota_handle;
                    uint8_t stop = false;
                    ota_partition = esp_ota_get_next_update_partition(NULL);
                    esp_ota_begin(ota_partition, 0, &ota_handle);
                    while (written < size) {
                        uint16_t to_read = (size - written) > sizeof(rx_buf) ? sizeof(rx_buf) : (size - written);
                        if (yielding_read(client_socket, rx_buf, to_read)) {
                            stop = true;
                            break;
                        }
                        written += to_read;
                        esp_ota_write(ota_handle, rx_buf, to_read);
                        
                    }
                    if (stop) {
                        break;
                    }
                    esp_ota_end(ota_handle);
                    esp_ota_set_boot_partition(ota_partition);
                    uint8_t ack = 1;
                    send(client_socket, &ack, 1, 0);
                    printf("Update done, restarting\n");
                    vTaskDelay(2000);
                    esp_restart();
                }
            }

            vTaskDelay(10);
        }// handle client loop
    }// accept client loop
}

void got_ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    struct sockaddr_in tcpServerAddr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(TCP_PORT)
    };
    mdns_init();
    char hostname[16];
    sprintf(hostname, "uwb_beacon%d", node_id);
    mdns_hostname_set(hostname);

    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    bind(tcp_socket, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr));
    listen(tcp_socket, 1);
    fcntl(tcp_socket, F_SETFL, O_NONBLOCK);
    wifi_connected = 1;
}

void wifi_disconnected_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if (wifi_connected) {
        close(tcp_socket);
        mdns_free();
    }
    wifi_connected = 0;

    static uint8_t ping = 0;
    if (ping) {
        memcpy(wifi_config.sta.ssid, user_ssid, sizeof(user_ssid));
        memcpy(wifi_config.sta.password, user_password, sizeof(user_password));
        ping = 0;
    } else {
        memcpy(wifi_config.sta.ssid, DEFAULT_SSID, sizeof(DEFAULT_SSID));
        memcpy(wifi_config.sta.password, DEFAULT_PASSWORD, sizeof(DEFAULT_PASSWORD));
        ping = 1;
    }
    
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect(); // a failed connect sends a disconnected event
}

void init_wifi () {
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    nvs_handle_t nvs;
    uint err, len;
    len = 255;
    err = nvs_open("storage", NVS_READWRITE, &nvs);
    err |= nvs_get_blob(nvs, "user_ssid", &user_ssid, &len);
    err |= nvs_get_blob(nvs, "user_password", &user_password, &len);
    if (err != ESP_OK) {
        nvs_set_blob(nvs, "user_ssid", DEFAULT_SSID, sizeof(DEFAULT_SSID));
        nvs_set_blob(nvs, "user_password", DEFAULT_PASSWORD, sizeof(DEFAULT_PASSWORD));
        memcpy(user_ssid, DEFAULT_SSID, sizeof(DEFAULT_SSID));
        memcpy(user_password, DEFAULT_PASSWORD, sizeof(DEFAULT_PASSWORD));
    }

    //ESP_ERROR_CHECK(err);
    memcpy(wifi_config.sta.ssid, user_ssid, sizeof(user_ssid));
    memcpy(wifi_config.sta.password, user_password, sizeof(user_password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_country_code("SE", true));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_disconnected_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_handler, NULL, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 5000, NULL, 2, NULL);
}