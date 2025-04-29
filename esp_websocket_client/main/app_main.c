/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_crt_bundle.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_websocket_client.h"
#include "mbedtls/sha256.h"
#include "mbedtls/base64.h"
#include <cJSON.h>
#include "esp_sntp.h"
#include "esp_netif_sntp.h"

#define NO_DATA_TIMEOUT_SEC 5
static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;
static esp_websocket_client_handle_t client;

#define APPID "7ce43e2b"
#define API_KEY "3c3db0ed324eaac4dac42ff7a54555cd"
#define API_SECRET "YmYwMDQ4NTVhNTE5NWM1NDBjN2JhNzBl"
#define HOST "spark-api.xf-yun.com"
#define PATH "/v4.0/chat"
#define DOMAIN "4.0Ultra"
static const char *TAG = "XF_SIMPLE";

// 全局状态标记
static bool response_received = false;
static char last_answer[512] = {0};

// 同步网络时间（必须调用）
static void sync_network_time()
{
    setenv("TZ", "UTC+8", 1); // 设置时区
    tzset();
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);

    int retry = 0;
    while (esp_netif_sntp_sync_wait(2000) == ESP_ERR_TIMEOUT && retry++ < 15)
    {
        printf("等待时间同步...\n");
    }
}
// 生成RFC1123格式的时间戳
static void get_rfc1123_date(char *buf, size_t len)
{
    time_t now;
    struct tm tm_time;
    time(&now);
    gmtime_r(&now, &tm_time);
    strftime(buf, len, "%a, %d %b %Y %H:%M:%S GMT", &tm_time);
}

// HMAC-SHA256签名生成
static void hmac_sha256(const char *key, const char *data, uint8_t *digest)
{
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, (const uint8_t *)key, strlen(key));
    mbedtls_md_hmac_update(&ctx, (const uint8_t *)data, strlen(data));
    mbedtls_md_hmac_finish(&ctx, digest);
    mbedtls_md_free(&ctx);
}
static void url_encode(const char *src, char *dst, size_t max_len)
{
    const char *hex = "0123456789ABCDEF";
    size_t len = strlen(src);
    size_t pos = 0;

    for (size_t i = 0; i < len && pos < max_len - 1; i++)
    {
        unsigned char c = src[i];

        // 特殊处理空格 -> '+'
        if (c == ' ')
        {
            dst[pos++] = '+';
        }
        // 保留字符直接保留
        else if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~')
        {
            dst[pos++] = c;
        }
        // 其他字符转义为 %XX
        else
        {
            if (pos + 3 > max_len)
                break;
            dst[pos++] = '%';
            dst[pos++] = hex[(c >> 4) & 0xF];
            dst[pos++] = hex[c & 0xF];
        }
    }
    dst[pos] = '\0';
}
// 生成认证URL
static char *generate_ws_url()
{
    char date[64];
    // time_t now = 0;
    // struct tm timeinfo = {0};
    // setenv("TZ", "CST-8", 1);
    // tzset();
    // localtime_r(&now, &timeinfo);

    get_rfc1123_date(date, sizeof(date));
    // 构造签名原始字符串
    char signature_str[256];
    snprintf(signature_str, sizeof(signature_str),
             "host: %s\ndate: %s\nGET %s HTTP/1.1", HOST, date, PATH);

    // 计算HMAC-SHA256
    uint8_t digest[32];
    hmac_sha256(API_SECRET, signature_str, digest);

    // Base64编码签名
    char signature_b64[64];
    size_t olen;
    mbedtls_base64_encode((uint8_t *)signature_b64, sizeof(signature_b64), &olen,
                          digest, sizeof(digest));

    // 构造授权字符串（无需额外编码）
    char authorization_origin[256];
    snprintf(authorization_origin, sizeof(authorization_origin),
             "api_key=\"%s\", algorithm=\"hmac-sha256\", "
             "headers=\"host date request-line\", signature=\"%s\"",
             API_KEY, signature_b64);

    // 仅对授权字符串进行Base64编码（Python对应步骤）
    char authorization[256];
    size_t auth_len;
    mbedtls_base64_encode((uint8_t *)authorization, sizeof(authorization), &auth_len,
                          (const uint8_t *)authorization_origin, strlen(authorization_origin));

    // URL组件编码（与Python的urlencode一致）
    ESP_LOGI(TAG, "date=%s", date);
    char auth_enc[256], host_enc[64], date_enc[64];
    url_encode(authorization, auth_enc, sizeof(auth_enc));
    url_encode(HOST, host_enc, sizeof(host_enc));
    url_encode(date, date_enc, sizeof(date_enc));
    // 构造最终URL
    char *url = malloc(512);
    snprintf(url, 512, "wss://%s%s?authorization=%s&date=%s&host=%s",
             HOST, PATH, auth_enc, date_enc, host_enc);
    return url;
}

// 构造请求JSON
static char *build_request_json(const char *query)
{
    cJSON *root = cJSON_CreateObject();

    // Header
    cJSON *header = cJSON_AddObjectToObject(root, "header");
    cJSON_AddStringToObject(header, "app_id", APPID);
    cJSON_AddStringToObject(header, "uid", "1234");

    // Parameter
    cJSON *parameter = cJSON_AddObjectToObject(root, "parameter");
    cJSON *chat = cJSON_AddObjectToObject(parameter, "chat");
    cJSON_AddStringToObject(chat, "domain", DOMAIN);
    cJSON_AddNumberToObject(chat, "temperature", 0.5);
    cJSON_AddNumberToObject(chat, "max_tokens", 4096);

    // Payload
    cJSON *payload = cJSON_AddObjectToObject(root, "payload");
    cJSON *message = cJSON_AddObjectToObject(payload, "message");
    cJSON *text = cJSON_AddArrayToObject(message, "text");

    cJSON *content = cJSON_CreateObject();
    cJSON_AddStringToObject(content, "role", "user");
    cJSON_AddStringToObject(content, "content", query);
    cJSON_AddItemToArray(text, content);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

// WebSocket事件处理
static void websocket_event_handler(void *arg, esp_event_base_t base,
                                    int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to server");
        // 发送请求数据
        char *request = build_request_json("今天星期几");
        esp_websocket_client_send_text(client, request, strlen(request), portMAX_DELAY);
        free(request);
        break;

    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x1)
        {
            ESP_LOGI(TAG, "Received: %.*s", data->data_len, (char *)data->data_ptr);
        }
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected");
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "错误: %s", (char *)data->data_ptr);
        break;
    }
}

void websocket_app_start()
{
    char *url = generate_ws_url();
    ESP_LOGI(TAG, "WebSocket URL: %s", url);

    // 配置WebSocket客户端
    esp_websocket_client_config_t config = {
        .uri = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .network_timeout_ms = 10000,
        .pingpong_timeout_sec = 5};

    client = esp_websocket_client_init(&config);
    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY,
                                  websocket_event_handler, NULL);

    // 启动连接
    esp_err_t ret = esp_websocket_client_start(client);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Connection failed: %s", esp_err_to_name(ret));
        free(url);
        return;
    }
    if (esp_websocket_client_is_connected(client) == ESP_OK)
    {
        ESP_LOGI(TAG, "连接成功，发送出错");
    }
    // char *request = build_request_json("你是谁");
    // ESP_LOGI(TAG, "%s", request);
    // esp_websocket_client_send_text(client, request, strlen(request), portMAX_DELAY);
    // free(request);
    free(url);
}
//}
// static const char *TAG = "mqtt_example";

// static void log_error_if_nonzero(const char *message, int error_code)
// {
//     if (error_code != 0)
//     {
//         ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
//     }
// }

// /*
//  * @brief Event handler registered to receive MQTT events
//  *
//  *  This function is called by the MQTT client event loop.
//  *
//  * @param handler_args user data registered to the event.
//  * @param base Event base for the handler(always MQTT Base in this example).
//  * @param event_id The id for the received event.
//  * @param event_data The data for the event, esp_mqtt_event_handle_t.
//  */
// static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
// {
//     ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
//     esp_mqtt_event_handle_t event = event_data;
//     esp_mqtt_client_handle_t client = event->client;
//     int msg_id;
//     switch ((esp_mqtt_event_id_t)event_id)
//     {
//     case MQTT_EVENT_CONNECTED:
//         ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//         msg_id = esp_mqtt_client_publish(client, "/k1kwkzvaqyl/LED2/user/setled", "1", 1, 1, 0);
//         ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

//         // msg_id = esp_mqtt_client_subscribe(client, "/k1kwkzvaqyl/LED2/user/setled", 0);
//         // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//         // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
//         // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//         break;
//     case MQTT_EVENT_DISCONNECTED:
//         ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
//         break;

//     case MQTT_EVENT_SUBSCRIBED:
//         ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//         // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
//         // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
//         break;
//     case MQTT_EVENT_UNSUBSCRIBED:
//         ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
//         break;
//     case MQTT_EVENT_PUBLISHED:
//         ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
//         break;
//     case MQTT_EVENT_DATA: // 订阅主题后数据触发事件
//         ESP_LOGI(TAG, "MQTT_EVENT_DATA");
//         printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
//         printf("DATA=%.*s\r\n", event->data_len, event->data);
//         break;
//     case MQTT_EVENT_ERROR:
//         ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
//         if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
//         {
//             log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
//             log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
//             log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
//             ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
//         }
//         break;
//     default:
//         ESP_LOGI(TAG, "Other event id:%d", event->event_id);
//         break;
//     }
// }

// static void mqtt_app_start(void)
// {
//     esp_mqtt_client_config_t mqtt_cfg = {
//         .broker.address.uri = CONFIG_BROKER_URL,
//         .credentials.client_id = "k1kwkzvaqyl.LED2|securemode=2,signmethod=hmacsha256,timestamp=1737165666091|",
//         .credentials.username = "LED2&k1kwkzvaqyl",
//         .credentials.authentication.password = "090bda3f0c8c9b73504b9c1c567adeedcab55ece27f1daed81ffffc1ad4909d6",
//         .credentials.authentication.key = "k1kwkzvaqyl",
//         .credentials.authentication.key_password = "8784820ab583a179a0e35205c2c121ae",

//     };
// #if CONFIG_BROKER_URL_FROM_STDIN
//     char line[128];

//     if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0)
//     {
//         int count = 0;
//         printf("Please enter url of mqtt broker\n");
//         while (count < 128)
//         {
//             int c = fgetc(stdin);
//             if (c == '\n')
//             {
//                 line[count] = '\0';
//                 break;
//             }
//             else if (c > 0 && c < 127)
//             {
//                 line[count] = c;
//                 ++count;
//             }
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }
//         mqtt_cfg.broker.address.uri = line;
//         printf("Broker url: %s\n", line);
//     }
//     else
//     {
//         ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
//         abort();
//     }
// #endif /* CONFIG_BROKER_URL_FROM_STDIN */

//     esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
//     /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
//     esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
//     esp_mqtt_client_start(client);
// }

void app_main(void)
{
    // ESP_LOGI(TAG, "[APP] Startup..");
    // ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    // ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    // esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    // esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    // esp_log_level_set("transport", ESP_LOG_VERBOSE);
    // esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    sync_network_time();
    // mqtt_app_start();
    websocket_app_start();
}
