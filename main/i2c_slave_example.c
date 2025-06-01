#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SLAVE_SCL_IO        15
#define I2C_SLAVE_SDA_IO        23
#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN    256
#define I2C_SLAVE_RX_BUF_LEN    256
#define ESP_SLAVE_ADDR          0x51

#define PP_API_VERSION          1
#define TRANSFER_BLOCK_SIZE     128
#define SHELL_BUFFER_MAX        256

static const char *TAG = "i2c_slave";

uint8_t  uart_app[];
size_t   uart_app_len;

typedef enum {
    COMMAND_NONE                   = 0x0000,
    COMMAND_INFO                   = 0x0001,
    COMMAND_APP_INFO               = 0x0002,
    COMMAND_APP_TRANSFER           = 0x0003,
    COMMAND_GETFEATURE_MASK        = 0x0004,
    COMMAND_GETFEAT_DATA_GPS       = 0x0005,
    COMMAND_GETFEAT_DATA_ORIENTATION = 0x0006,
    COMMAND_GETFEAT_DATA_ENVIRONMENT  = 0x0007,
    COMMAND_GETFEAT_DATA_LIGHT     = 0x0008,
    COMMAND_SHELL_PPTOMOD_DATA     = 0x0009,
    COMMAND_SHELL_MODTOPP_DATA_SIZE = 0x000A,
    COMMAND_SHELL_MODTOPP_DATA     = 0x000B,

    COMMAND_UART_REQUESTDATA_SHORT = 0x7F01,
    COMMAND_UART_REQUESTDATA_LONG  = 0x7F02,
    COMMAND_UART_BAUDRATE_INC      = 0x7F03,
    COMMAND_UART_BAUDRATE_DEC      = 0x7F04,
    COMMAND_UART_BAUDRATE_GET      = 0x7F05,
} Command;

typedef enum { UTILITIES = 0, RX, TX, DEBUG, HOME } app_location_t;

#pragma pack(push,1)
typedef struct {
    uint32_t api_version;
    uint32_t module_version;
    char     module_name[20];
    uint32_t application_count;
} device_info;

typedef struct {
    uint32_t      header_version;
    uint8_t       app_name[16];
    uint8_t       bitmap_data[32];
    uint32_t      icon_color;
    app_location_t menu_location;
    uint32_t      binary_size;
} standalone_app_info;

typedef struct {
    uint16_t angle;
    int16_t  tilt;
} orientation_data_t;
#pragma pack(pop)

static uint8_t  shell_buffer[SHELL_BUFFER_MAX];
static size_t   shell_buffer_len = 0;

static uint16_t last_app_index  = 0;
static size_t   last_app_offset = 0;

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUDRATE      115200
#define UART_BUF_SIZE      1024
#define UART_QUEUE_SIZE    1024

static uint8_t  uart_queue[UART_QUEUE_SIZE];
static size_t   uart_queue_head = 0;
static size_t   uart_queue_tail = 0;

static const int baudrates[] = {50, 75, 110, 134, 150, 200, 300, 600,
    1200, 2400, 4800, 9600, 14400, 19200,
    28800, 38400, 57600, 115200, 230400,
    460800, 576000, 921600, 1843200, 3686400};
static int current_baud_index = 0;

static void i2c_slave_init(void);
static void uart_init(int baudrate);
static void uart_read_task(void *arg);

static void handle_command(uint16_t cmd,
                           uint8_t *req_buf, size_t req_len,
                           uint8_t *resp_buf, size_t *resp_len);

static void uart_queue_push(uint8_t byte) {
    size_t next = (uart_queue_head + 1) % UART_QUEUE_SIZE;
    if (next != uart_queue_tail) {
        uart_queue[uart_queue_head] = byte;
        uart_queue_head = next;
    }
}

static uint8_t uart_queue_pop(void) {
    if (uart_queue_head == uart_queue_tail) {
        return 0;
    }
    uint8_t b = uart_queue[uart_queue_tail];
    uart_queue_tail = (uart_queue_tail + 1) % UART_QUEUE_SIZE;
    return b;
}

static size_t uart_queue_available(void) {
    return (uart_queue_head + UART_QUEUE_SIZE - uart_queue_tail) % UART_QUEUE_SIZE;
}

static size_t read_app_block(uint16_t index, size_t offset, uint8_t *out_buf, size_t max_len) {
    if (index != 0) {
        return 0;
    }
    if (offset >= uart_app_len) {
        return 0;
    }
    size_t left = uart_app_len - offset;
    size_t to_copy = left < max_len ? left : max_len;
    memcpy(out_buf, uart_app + offset, to_copy);
    return to_copy;
}

static void handle_info(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    device_info dev = {
        .api_version      = PP_API_VERSION,
        .module_version   = 1,
    };
    strncpy(dev.module_name, "ESP32 Module", sizeof(dev.module_name));
    dev.application_count = 1;
    memcpy(resp, &dev, sizeof(dev));
    *resp_len = sizeof(dev);
}

static void handle_app_info(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req_len;
    uint16_t idx = req[2] | (req[3] << 8);
    if (idx != 0) {
        ESP_LOGW(TAG, "Invalid app index %u", idx);
    }
    standalone_app_info app = {0};
    app.header_version = 1;
    strncpy((char*)app.app_name, "UART", sizeof(app.app_name));
    memset(app.bitmap_data, 0xFF, sizeof(app.bitmap_data));
    app.icon_color    = 0x00FF00;
    app.menu_location = DEBUG;
    app.binary_size   = uart_app_len;
    memcpy(resp, &app, sizeof(app));
    *resp_len = sizeof(app);
}

static void handle_app_transfer(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req_len;
    uint16_t idx = req[2] | (req[3] << 8);
    uint16_t blk = req[4] | (req[5] << 8);
    last_app_index = idx;
    last_app_offset = (size_t)blk * TRANSFER_BLOCK_SIZE;
    size_t copied = read_app_block(last_app_index, last_app_offset, resp, TRANSFER_BLOCK_SIZE);
    *resp_len = copied;
}

static void handle_getfeature_mask(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint64_t features = 0;
    features |= (1ULL << 0);  // EXT_APP
    features |= (1ULL << 1);  // UART
    features |= (1ULL << 2);  // GPS
    features |= (1ULL << 3);  // ORIENTATION
    features |= (1ULL << 4);  // ENVIRONMENT
    features |= (1ULL << 5);  // LIGHT
    features |= (1ULL << 6);  // DISPLAY
    features |= (1ULL << 7);  // SHELL
    memcpy(resp, &features, sizeof(features));
    *resp_len = sizeof(features);
}

static void handle_getfeat_data_gps(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint8_t gps_data[16] = {0};
    memcpy(resp, gps_data, sizeof(gps_data));
    *resp_len = sizeof(gps_data);
}

static void handle_getfeat_data_orientation(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    orientation_data_t ori = { .angle = 1234, .tilt = 5678 };
    memcpy(resp, &ori, sizeof(ori));
    *resp_len = sizeof(ori);
}

static void handle_getfeat_data_environment(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    struct { float t,h,p; } env = { .t = 23.5f, .h = 55.0f, .p = 1013.2f };
    memcpy(resp, &env, sizeof(env));
    *resp_len = sizeof(env);
}

static void handle_getfeat_data_light(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint16_t light = 321;
    memcpy(resp, &light, sizeof(light));
    *resp_len = sizeof(light);
}

static void handle_shell_pptomod_data(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)resp; (void)resp_len;
    size_t payload = req_len - 2;
    if (payload > SHELL_BUFFER_MAX) payload = SHELL_BUFFER_MAX;
    memcpy(shell_buffer, &req[2], payload);
    shell_buffer_len = payload;

    // Log HEX
    char hexstr[3 * SHELL_BUFFER_MAX + 1] = {0};
    char *h = hexstr;
    for (size_t i = 0; i < payload; i++) {
        sprintf(h, "%02X ", shell_buffer[i]);
        h += 3;
    }
    ESP_LOGI(TAG, "Shell RX HEX: %s", hexstr);

    // Log ASCII
    char asciistr[SHELL_BUFFER_MAX + 1] = {0};
    size_t ai = 0;
    for (size_t i = 0; i < payload; i++) {
        uint8_t c = shell_buffer[i];
        if (c == '\n' || c == '\r') asciistr[ai++] = c;
        else if (isprint(c))          asciistr[ai++] = c;
    }
    asciistr[ai] = '>';
    ESP_LOGI(TAG, "Shell RX ASCII: %s", asciistr);
}

static void handle_shell_modtopp_data_size(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint16_t size = (uint16_t)shell_buffer_len;
    resp[0] = size & 0xFF;
    resp[1] = (size >> 8) & 0xFF;
    *resp_len = 2;
}

static void handle_shell_modtopp_data(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint8_t block = (shell_buffer_len > 64) ? 64 : shell_buffer_len;
    uint8_t flag  = (shell_buffer_len > 64) ? 0x80 : 0x00;
    resp[0] = flag | (block & 0x7F);
    memcpy(&resp[1], shell_buffer, block);
    *resp_len = (size_t)(1 + block);
}

static void handle_uart_request_short(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    size_t available = uart_queue_available();
    size_t to_send   = available < 4 ? available : 4;
    uint8_t header   = (available > 4 ? 0x80 : 0x00) | (uint8_t)to_send;
    resp[0] = header;
    for (size_t i = 0; i < to_send; i++) {
        resp[i + 1] = uart_queue_pop();
    }
    *resp_len = (size_t)(1 + to_send);
    ESP_LOGI(TAG, "Sent SHORT UART data (%u bytes)", (unsigned)to_send);
}

static void handle_uart_request_long(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    size_t available = uart_queue_available();
    size_t to_send   = available < 127 ? available : 127;
    uint8_t header   = (available > 127 ? 0x80 : 0x00) | (uint8_t)to_send;
    resp[0] = header;
    for (size_t i = 0; i < to_send; i++) {
        resp[i + 1] = uart_queue_pop();
    }
    *resp_len = (size_t)(1 + to_send);
    ESP_LOGI(TAG, "Sent LONG UART data (%u bytes)", (unsigned)to_send);
}

static void handle_uart_baud_inc(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len; (void)resp; (void)resp_len;
    if (current_baud_index < (int)(sizeof(baudrates)/sizeof(baudrates[0])) - 1) {
        current_baud_index++;
        uart_init(baudrates[current_baud_index]);
        ESP_LOGI(TAG, "Baudrate increased to %d", baudrates[current_baud_index]);
    }
}

static void handle_uart_baud_dec(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len; (void)resp; (void)resp_len;
    if (current_baud_index > 0) {
        current_baud_index--;
        uart_init(baudrates[current_baud_index]);
        ESP_LOGI(TAG, "Baudrate decreased to %d", baudrates[current_baud_index]);
    }
}

static void handle_uart_baud_get(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    int br = baudrates[current_baud_index];
    memcpy(resp, &br, sizeof(br));
    *resp_len = sizeof(br);
    ESP_LOGI(TAG, "Returned baudrate %d", br);
}

static void handle_command(uint16_t cmd,
                           uint8_t *req_buf, size_t req_len,
                           uint8_t *resp_buf, size_t *resp_len){
    *resp_len = 0;

    switch (cmd) {
        case COMMAND_INFO:
            handle_info(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_APP_INFO:
            handle_app_info(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_APP_TRANSFER:
            handle_app_transfer(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_GETFEATURE_MASK:
            handle_getfeature_mask(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_GETFEAT_DATA_GPS:
            handle_getfeat_data_gps(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_GETFEAT_DATA_ORIENTATION:
            handle_getfeat_data_orientation(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_GETFEAT_DATA_ENVIRONMENT:
            handle_getfeat_data_environment(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_GETFEAT_DATA_LIGHT:
            handle_getfeat_data_light(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_SHELL_PPTOMOD_DATA:
            handle_shell_pptomod_data(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_SHELL_MODTOPP_DATA_SIZE:
            handle_shell_modtopp_data_size(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_SHELL_MODTOPP_DATA:
            handle_shell_modtopp_data(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_UART_REQUESTDATA_SHORT:
            handle_uart_request_short(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_UART_REQUESTDATA_LONG:
            handle_uart_request_long(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_UART_BAUDRATE_INC:
            handle_uart_baud_inc(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_UART_BAUDRATE_DEC:
            handle_uart_baud_dec(req_buf, req_len, resp_buf, resp_len);
            break;
        case COMMAND_UART_BAUDRATE_GET:
            handle_uart_baud_get(req_buf, req_len, resp_buf, resp_len);
            break;
        default:
            ESP_LOGW(TAG, "Unhandled command 0x%04X", cmd);
            break;
    }
}

static void i2c_slave_init(void){
    ESP_LOGI(TAG, "Init slave I2C...");
    i2c_config_t conf = {
        .sda_io_num    = I2C_SLAVE_SDA_IO,
        .scl_io_num    = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode          = I2C_MODE_SLAVE,
        .slave = {
            .slave_addr    = ESP_SLAVE_ADDR,
            .maximum_speed = 100000
        }
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    ESP_LOGI(TAG, "I2C slave ready @0x%02X", ESP_SLAVE_ADDR);
}

static void uart_init(int baudrate){
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_set_pin(UART_PORT_NUM,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
}

static void uart_read_task(void *arg){
    uint8_t buf[128];
    while (true) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            uart_queue_push(buf[i]);
        }
    }
}

void app_main(void){
    i2c_slave_init();
    ESP_LOGI(TAG, "I2C slave initialized, waiting for commands...");

    uart_init(UART_BAUDRATE);
    xTaskCreate(uart_read_task, "uart_read_task", 2048, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART initialized (baud %d), UART read task started.", UART_BAUDRATE);

    uint8_t recv_buf[512];
    uint8_t resp_buf[TRANSFER_BLOCK_SIZE + 2];
    size_t  resp_len = 0;

    while (true) {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, recv_buf, sizeof(recv_buf), pdMS_TO_TICKS(500));
        if (len < 2) {
            continue;
        }

        uint16_t cmd = (uint16_t)recv_buf[0] | ((uint16_t)recv_buf[1] << 8);
        resp_len = 0;
        memset(resp_buf, 0, sizeof(resp_buf));

        handle_command(cmd, recv_buf, (size_t)len, resp_buf, &resp_len);

        if (resp_len) {
            int sent = i2c_slave_write_buffer(I2C_SLAVE_NUM, resp_buf, resp_len, pdMS_TO_TICKS(500));
            ESP_LOGI(TAG, "Sent %d bytes in response to cmd 0x%04X", sent, cmd);
        }
    }
}
