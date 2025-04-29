
/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <ctype.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"

#include "esp_tls.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "cJSON.h"
#include "esp_http_client.h"
#include <stdio.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "cJSON.h"
char *TAG = "AI_Ret";

#define UART_RX_BUF_SIZE 1024 // 串口接收缓冲区大小

#define ECHO_TEST_TXD (GPIO_NUM_17)
#define ECHO_TEST_RXD (GPIO_NUM_16)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM (UART_NUM_1)

#define BUF_SIZE (1024)

static char *response_buffer = NULL;

static esp_err_t _http_event_xunfeihandler(esp_http_client_event_t *evt)
{
    char sse_buffer[512] = {0}; // 静态缓冲区累积分块数据
    size_t sse_buffer_len = 0;
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
    {
        // 1. 将新数据追加到缓冲区
        size_t copy_len = MIN(sizeof(sse_buffer) - sse_buffer_len - 1, evt->data_len);
        memcpy(sse_buffer + sse_buffer_len, evt->data, copy_len);
        sse_buffer_len += copy_len;
        sse_buffer[sse_buffer_len] = '\0';
        char *tokens = strstr(sse_buffer, "total_tokens");
        char *content = strstr(sse_buffer, "content");
        if (tokens != NULL) // 最后一包处理
        {
            tokens = strtok(tokens, "}") + 14;
            ESP_LOGI(TAG, "total_tokens消耗:%s", tokens);
            return 0;
        }
        if (content != NULL)
        {
            content = strtok(content, "}") + 9;
            ESP_LOGI(TAG, "%s", content);
        }

            break;
    }

    case HTTP_EVENT_DISCONNECTED:
    {
        sse_buffer_len = 0;
            break;
    }

    default:
            break;
    }
    return ESP_OK;
}
// 处理太慢
static esp_err_t _http_event_dobaohandler(esp_http_client_event_t *evt)
{
    static char sse_line[512] = {0};
    static size_t sse_pos = 0;

    if (evt->event_id == HTTP_EVENT_ON_DATA)
    {
        for (int i = 0; i < evt->data_len; i++)
        {
            char c = ((char *)evt->data)[i];

            // 采集整行数据
            if (c == '\n')
            {
                sse_line[sse_pos] = '\0';
                sse_pos = 0;

                // 检测有效 SSE 行
                if (strncmp(sse_line, "data:", 5) == 0)
                {
                    char *data_start = sse_line + 5;
                    if (strstr(data_start, "[DONE]") != NULL)
                    {
                        return ESP_OK;
                    }
                    while (*data_start == ' ' || *data_start == '\t')
                        data_start++; // 兼容多空格

                    cJSON *root = cJSON_Parse(data_start);
                    if (!root)
                    {
                        ESP_LOGE(TAG, "解析失败! 可能原因: %s", cJSON_GetErrorPtr());
                        const char *error_pos = cJSON_GetErrorPtr();
                        if (error_pos)
                        {
                            ESP_LOGE(TAG, "错误上下文: ...%.32s...", error_pos - 16); // 打印错误周围上下文
                        }
                            return ESP_FAIL;
                    }
                    // else
                    // {
                    //     ESP_LOGI(TAG, "%s", cJSON_Print(root));
                    // }

                    cJSON *choices = cJSON_GetObjectItemCaseSensitive(root, "choices");
                    cJSON *usage = cJSON_GetObjectItemCaseSensitive(root, "usage");
                    if (cJSON_IsArray(choices))
                    {
                        cJSON *item;
                        cJSON_ArrayForEach(item, choices)
                        {
                            // 1. 提取 "delta" 对象，并检查是否存在
                            cJSON *message_obj = cJSON_GetObjectItemCaseSensitive(item, "delta");
                            if (message_obj == NULL || !cJSON_IsObject(message_obj))
                            {
                                ESP_LOGE(TAG, "Invalid or missing 'delta' field");
                                continue; // 跳过无效条目
                            }

                            // 2. 检查是否存在 "reasoning_content" 字段
                            cJSON *role = cJSON_GetObjectItemCaseSensitive(message_obj, "reasoning_content");
                            if (role != NULL && cJSON_IsString(role))
                            {
                                ESP_LOGI(TAG, "深度思考:%s", role->valuestring);
                            }

                            // 3. 检查是否存在 "content" 字段，并确保非空
                            cJSON *content = cJSON_GetObjectItemCaseSensitive(message_obj, "content");
                            if (content != NULL && cJSON_IsString(content) &&
                                content->valuestring != NULL && strlen(content->valuestring) > 0)
                            {
                                ESP_LOGI(TAG, "最终回答:%s", content->valuestring);
                            }
                        }
                    }

                    if (cJSON_IsObject(usage))
                    {
                        cJSON *total_tokens = cJSON_GetObjectItemCaseSensitive(usage, "total_tokens");
                        if (cJSON_IsNumber(total_tokens))
                        {
                            ESP_LOGI(TAG, "Total Tokens: %d\n", total_tokens->valueint);
                        }
                    }

                    cJSON_Delete(root);
                }
            }
            else if (c != '\r' && sse_pos < sizeof(sse_line) - 1)
            {
                sse_line[sse_pos++] = c;
            }
        }
    }
    return ESP_OK;
}

char *build_json_xunfeipayload(const char *user_input)
{
    // 创建根JSON对象
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", "4.0Ultra");

    // 添加messages数组
    cJSON *messages = cJSON_AddArrayToObject(root, "messages");
    cJSON *message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "role", "user");
    cJSON_AddStringToObject(message, "content", user_input);
    cJSON_AddItemToArray(messages, message);

    // 添加stream字段
    cJSON_AddTrueToObject(root, "stream"); // 或 cJSON_AddFalseToObject

    // 添加tools数组
    cJSON *tools = cJSON_AddArrayToObject(root, "tools");
    cJSON *tool = cJSON_CreateObject();
    cJSON_AddStringToObject(tool, "type", "web_search");

    // 添加web_search子对象
    cJSON *web_search = cJSON_CreateObject();
    cJSON_AddTrueToObject(web_search, "enable");
    cJSON_AddTrueToObject(web_search, "show_ref_label");
    cJSON_AddStringToObject(web_search, "search_mode", "deep");
    cJSON_AddItemToObject(tool, "web_search", web_search);

    cJSON_AddItemToArray(tools, tool);

    // 添加max_tokens字段
    cJSON_AddNumberToObject(root, "max_tokens", 2048);

    // 生成JSON字符串
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

char *build_json_dobaopayload(const char *user_input)
{
    // 创建根JSON对象
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", "deepseek-r1-250120");

    // 添加messages数组
    cJSON *messages = cJSON_AddArrayToObject(root, "messages");
    cJSON *message = cJSON_CreateObject();
    cJSON_AddStringToObject(message, "role", "user");
    cJSON_AddStringToObject(message, "content", user_input);
    cJSON_AddItemToArray(messages, message);

    // 添加stream字段
    cJSON_AddTrueToObject(root, "stream"); // 或 cJSON_AddFalseToObject
                                           // cJSON_AddTrueToObject(root, "stream_options.include_usage");

    cJSON *stream_options = cJSON_AddObjectToObject(root, "stream_options");
    cJSON_AddTrueToObject(stream_options, "include_usage"); // 新增 include_usage: true
    // 添加web_search子对象
    // cJSON *web_search = cJSON_CreateObject();
    // cJSON_AddTrueToObject(web_search, "enable");
    // cJSON_AddTrueToObject(web_search, "show_ref_label");
    // cJSON_AddStringToObject(web_search, "search_mode", "deep");
    // cJSON_AddItemToObject(tool, "web_search", web_search);

    // cJSON_AddItemToArray(tools, tool);

    // 添加max_tokens字段
    cJSON_AddNumberToObject(root, "max_tokens", 2048);

    // 生成JSON字符串
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

void send_http_xufeirequest(const char *user_input)
{
    // 1. 构造 JSON 请求体
    char *post_data = build_json_dobaopayload(user_input);
    // ESP_LOGI(TAG, "send json: %s", post_data);
    // 分配内存

    if (post_data == NULL)
    {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }

    // 2. 配置 HTTP 客户端
    esp_http_client_config_t config = {
        .url = "https://spark-api-open.xf-yun.com/v1/chat/completions",
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_xunfeihandler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 20000};
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // 3. 设置请求头和 POST 数据
    esp_http_client_set_header(client, "Authorization", "Bearer pVZuNPMIUAKkRyFiBYIs:lRFiEmxcrBQKWmosKcDI");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // 4. 发送请求
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        //        ESP_LOGI(TAG, "HTTP 状态码: %d", esp_http_client_get_status_code(client));
    }
    else
    {
        ESP_LOGE(TAG, "请求失败: %s", esp_err_to_name(err));
    }

    // 5. 清理资源
    esp_http_client_cleanup(client);
    free(post_data);
}
void send_http_dobaorequest(const char *user_input)
{
    // 1. 构造 JSON 请求体
    char *post_data = build_json_dobaopayload(user_input);
    ESP_LOGI(TAG, "send json: %s", post_data);
    // 分配内存

    if (post_data == NULL)
    {
        ESP_LOGE(TAG, "内存分配失败");
        return;
    }

    // 2. 配置 HTTP 客户端
    esp_http_client_config_t config = {
        .url = "https://ark.cn-beijing.volces.com/api/v3/chat/completions",
        .method = HTTP_METHOD_POST,
        .event_handler = _http_event_dobaohandler,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 30000};
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // 3. 设置请求头和 POST 数据
    esp_http_client_set_header(client, "Authorization", "Bearer 48f6e428-af6d-4712-b982-20890e5d9349");
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // 4. 发送请求
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP 状态码: %d", esp_http_client_get_status_code(client));
    }
    else
    {
        ESP_LOGE(TAG, "请求失败: %s", esp_err_to_name(err));
    }

    // 5. 清理资源
    esp_http_client_cleanup(client);
    free(post_data);
}

void http_request_task(void *pvParameters)
{
    char *user_input = (char *)pvParameters;
    if (!user_input)
    {
        vTaskDelete(NULL);
        return;
    }

    // 发送 HTTP 请求（复用你的 send_http_request 函数）
    //    send_http_xufeirequest(user_input);
    send_http_dobaorequest(user_input);
    // 清理资源
    free(user_input);
    vTaskDelete(NULL);
}
void uart_listen_task(void *pvParameters)
{
    uint8_t *data = malloc(UART_RX_BUF_SIZE + 1); // 堆分配缓冲区
    if (!data)
    {
        ESP_LOGE(TAG, "内存分配失败");
        vTaskDelete(NULL);
    }

    while (1)
    {
        // 非阻塞读取串口数据
        int len = uart_read_bytes(UART_NUM_1, data, UART_RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            data[len] = '\0'; // 安全终止字符串
            ESP_LOGI(TAG, "收到输入: %s", data);

            // 将数据复制到新内存并触发 HTTP 请求任务
            char *user_input = strndup((char *)data, len);
            if (user_input)
            {
                xTaskCreate(http_request_task, "http_task", 8192, user_input, 5, NULL);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(data); // 理论上不会执行到这里
}

void main_task(void *pvParameters)
{
    // 初始化串口
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));

    // 创建串口监听任务（独立栈）
    xTaskCreate(uart_listen_task, "uart_task", 4096, NULL, 5, NULL);

    // 主任务可继续处理其他逻辑或挂起
    vTaskDelete(NULL); // 删除当前任务，保留串口监听任务运行
}
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(main_task, "main_task", 6144, NULL, 5, NULL);
}
