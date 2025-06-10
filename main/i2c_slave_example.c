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
#define MAX_APPLICATIONS        1

static const char *TAG = "i2c_slave";

#define UART_RX_PIN          16 // GPIO for UART RX, just to compile, at this moment it's not used

//TODO, replace with external UART app
// This is a simulated UART application for testing purposes.
// In a real application, this would be replaced with the actual UART app binary.
static uint8_t uart_app[] = {
    // Header PortaPack App 
    0x50, 0x50, 0x41, 0x50,  // "PPAP" - PortaPack App signature
    0x01, 0x00, 0x00, 0x00,  // Version 1
    0x55, 0x41, 0x52, 0x54,  // "UART"
    0x00, 0x00, 0x00, 0x00,  // Padding
    
    // Simulated binary code for the UART app
    0xE9, 0x00, 0x00, 0x00,  // Jump instruction
    0x48, 0x65, 0x6C, 0x6C,  // "Hell"
    0x6F, 0x20, 0x50, 0x6F,  // "o Po"
    0x72, 0x74, 0x61, 0x50,  // "rtaP"
    0x61, 0x63, 0x6B, 0x21,  // "ack!"
    0x00, 0x00, 0x00, 0x00,  // Padding
    
    // more dummy instructions
    0x90, 0x90, 0x90, 0x90,  // NOPs
    0xC3, 0x00, 0x00, 0x00,  // RET instruction
    
    // fill the rest of the app with zeroes until the block size
    [36 ... TRANSFER_BLOCK_SIZE - 1] = 0x00
};


// fill the rest of the app with dummy data


static size_t uart_app_len = sizeof(uart_app);

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

static uint8_t shell_buffer[SHELL_BUFFER_MAX];
static size_t shell_buffer_len = 0;

#define UART_PORT_NUM      UART_NUM_1 
#define UART_BAUDRATE      115200
#define UART_BUF_SIZE      1024
#define UART_QUEUE_SIZE    2048

static uint8_t uart_queue[UART_QUEUE_SIZE];
static size_t uart_queue_head = 0;
static size_t uart_queue_tail = 0;

static const int baudrates[] = {
    50, 75, 110, 134, 150, 200, 300, 600,
    1200, 2400, 4800, 9600, 14400, 19200,
    28800, 38400, 57600, 115200, 230400,
    460800, 576000, 921600, 1843200, 3686400
};
static int current_baud_index = 0;
static uint32_t current_baudrate = UART_BAUDRATE; 

static void i2c_slave_init(void);
static void uart_init(int baudrate);
static void uart_deinit(void);
static void uart_read_task(void *arg);
static void handle_command(uint16_t cmd,
                           uint8_t *req_buf, size_t req_len,
                           uint8_t *resp_buf, size_t *resp_len);

static void init_default_baudrate(void) {
    // seek the default baudrate in the predefined list
    for (int i = 0; i < sizeof(baudrates)/sizeof(baudrates[0]); i++) {
        if (baudrates[i] == UART_BAUDRATE) {
            current_baud_index = i;
            current_baudrate = UART_BAUDRATE;
            break;
        }
    }
    ESP_LOGI(TAG, "Default baudrate set to %d (index %d)", UART_BAUDRATE, current_baud_index);
}
// virtual UART queue management
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
    if (index >= MAX_APPLICATIONS) {
        ESP_LOGW(TAG, "read_app_block: Invalid index %u", index);
        return 0;
    }
    
    if (offset >= uart_app_len) {
        ESP_LOGD(TAG, "read_app_block: Offset %zu beyond app size %zu", 
                 offset, uart_app_len);
        return 0;
    }
    
    size_t left = uart_app_len - offset;
    size_t to_copy = left < max_len ? left : max_len;
    
    memcpy(out_buf, uart_app + offset, to_copy);
    
    ESP_LOGD(TAG, "read_app_block: index=%u, offset=%zu, copied=%zu", 
             index, offset, to_copy);
    
    return to_copy;
}

static void handle_info(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    
    device_info dev = {0};
    dev.api_version      = PP_API_VERSION;
    dev.module_version   = 1;
    strncpy(dev.module_name, "ESP32 Module", sizeof(dev.module_name)-1);
    dev.application_count = MAX_APPLICATIONS;  
    
    memcpy(resp, &dev, sizeof(dev));
    *resp_len = sizeof(dev);
    
    ESP_LOGI(TAG, "Sent device info: API v%lu, %lu applications", dev.api_version, dev.application_count);
}

static void handle_app_info(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    if (req_len < 4) {
        ESP_LOGW(TAG, "APP_INFO: insufficient data length %zu", req_len);
        *resp_len = 0;
        return;
    }
    
    uint16_t idx = (uint16_t)req[2] | ((uint16_t)req[3] << 8);
    
    if (idx != 0) {
        ESP_LOGW(TAG, "Invalid app index %u (only index 0 available)", idx);
        // TODO: investigate if we should return an error 
        // *resp_len = 0;  
        // return;
    }

    standalone_app_info app = {0};
    app.header_version = PP_API_VERSION;
    strncpy((char*)app.app_name, "UART", sizeof(app.app_name));
    
    uint8_t terminal_bitmap[32] = {
        0xFF, 0xFF, 0xFF, 0xFF,  // ââââââââââââââââ
        0x80, 0x00, 0x00, 0x01,  // â              â
        0x80, 0x24, 0x24, 0x01,  // â              â
        0x80, 0x00, 0x00, 0x01,  // â              â
        0x80, 0x48, 0x48, 0x01,  // â              â
        0x80, 0x00, 0x00, 0x01,  // â              â
        0x80, 0x92, 0x92, 0x01,  // â              â
        0xFF, 0xFF, 0xFF, 0xFF   // ââââââââââââââââ
    };
    memcpy(app.bitmap_data, terminal_bitmap, sizeof(app.bitmap_data));
    
    app.icon_color    = 0x00FF00;      // Green
    app.menu_location = DEBUG;         // Place, DEBUG, no UTILITIES
    app.binary_size   = (uint32_t)uart_app_len;
    
    memcpy(resp, &app, sizeof(app));
    *resp_len = sizeof(app);
    
    ESP_LOGI(TAG, "APP_INFO: Sent info for app 0 (UART, %u bytes)", 
             (unsigned)sizeof(app));
    ESP_LOGI(TAG, "Sent %zu bytes of APP_INFO for index %u (name=UART)", sizeof(app), idx);
}

static void handle_app_transfer(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    if (req_len < 6) {
        ESP_LOGW(TAG, "APP_TRANSFER request too short (%zu bytes)", req_len);
        *resp_len = 0;
        return;
    }
    
    uint16_t idx = (uint16_t)req[2] | ((uint16_t)req[3] << 8);
    uint16_t blk = (uint16_t)req[4] | ((uint16_t)req[5] << 8);
    
    if (idx >= MAX_APPLICATIONS) {
        ESP_LOGW(TAG, "Invalid app index %u for transfer (max: %d)", 
                 idx, MAX_APPLICATIONS - 1);
        *resp_len = 0;
        return;
    }
    
    size_t offset = (size_t)blk * TRANSFER_BLOCK_SIZE;
    size_t copied = read_app_block(idx, offset, resp, TRANSFER_BLOCK_SIZE);
    *resp_len = copied;
    
    ESP_LOGI(TAG, "APP_TRANSFER: app=%u, block=%u, offset=%zu, sent=%zu bytes",
             idx, blk, offset, copied);
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
    ESP_LOGI(TAG, "Sent GETFEATURE_MASK (0x%016llX)", (long long)features);
}

static void handle_getfeat_data_gps(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    ESP_LOGI(TAG, "Handling GPS data request");
    (void)req; (void)req_len;
    uint8_t gps_data[16] = {0};
    memcpy(resp, gps_data, sizeof(gps_data));
    *resp_len = sizeof(gps_data);
    ESP_LOGI(TAG, "Sent GPS data (16 bytes)");
}

static void handle_getfeat_data_orientation(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    orientation_data_t ori = { .angle = 12, .tilt = 56 };
    memcpy(resp, &ori, sizeof(ori));
    *resp_len = sizeof(ori);
    ESP_LOGI(TAG, "Sent orientation data (angle=%u, tilt=%d)", ori.angle, ori.tilt);
}

static void handle_getfeat_data_environment(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    struct { float t, h, p; } env = { .t = 23.5f, .h = 55.0f, .p = 1013.2f };
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
    ESP_LOGI(TAG, "Handling shell data from PPTOMOD (%zu bytes)", req_len);
    if (req_len > 2) {
        size_t payload = req_len - 2;  // Descontar los 2 bytes del comando
        if (payload > SHELL_BUFFER_MAX) payload = SHELL_BUFFER_MAX;
        
        memcpy(shell_buffer, &req[2], payload);
        shell_buffer_len = payload;

        // Log HEX 
        char hexstr[3 * 32 + 1] = {0};
        char *h = hexstr;
        size_t log_bytes = payload < 32 ? payload : 32;
        for (size_t i = 0; i < log_bytes; i++) {
            sprintf(h, "%02X ", shell_buffer[i]);
            h += 3;
        }
        ESP_LOGI(TAG, "Shell RX HEX (%zu bytes): %s%s", payload, hexstr, payload > 32 ? "..." : "");

        // Log ASCII
        char asciistr[SHELL_BUFFER_MAX + 1] = {0};
        size_t ai = 0;
        for (size_t i = 0; i < payload && ai < SHELL_BUFFER_MAX - 1; i++) {
            uint8_t c = shell_buffer[i];
            if (c == '\n' || c == '\r') {
                asciistr[ai++] = c;
            } else if (isprint(c)) {
                asciistr[ai++] = c;
            } else {
                //asciistr[ai++] = '.';  // its better to not include non-printable characters
            }
        }
        ESP_LOGI(TAG, "Shell RX ASCII: %s", asciistr);
    }
    
    // IMPORTANT: send ACK response
    resp[0] = 0x00;  // Status OK
    *resp_len = 1;   // Acknowledge the command
    
    ESP_LOGI(TAG, "Shell PPTOMOD processed %zu bytes, sent ACK", req_len > 2 ? req_len - 2 : 0);
}

static void handle_shell_modtopp_data_size(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint16_t size = (uint16_t)shell_buffer_len;
    resp[0] = size & 0xFF;
    resp[1] = (size >> 8) & 0xFF;
    *resp_len = 2;
    ESP_LOGI(TAG, "handle_shell_modtopp_data_size size: %zu", *resp_len);
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
    
    uart_deinit();  
    
    if (current_baud_index >= (int)(sizeof(baudrates)/sizeof(baudrates[0])) - 1) {
        current_baud_index = 0; 
    } else {
        current_baud_index++;
    }
    
    current_baudrate = baudrates[current_baud_index];
    uart_init(current_baudrate); // restart UART with new baudrate
    
    ESP_LOGI(TAG, "Baudrate increased to %lu", current_baudrate);
}

static void handle_uart_baud_dec(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len; (void)resp; (void)resp_len;
    
    uart_deinit();  // deinit UART
    
    if (current_baud_index <= 0) {
        current_baud_index = (int)(sizeof(baudrates)/sizeof(baudrates[0])) - 1;  // Ir al final si estamos en el primero
    } else {
        current_baud_index--;
    }
    
    current_baudrate = baudrates[current_baud_index];
    uart_init(current_baudrate);  // restart UART with new baudrate
    
    ESP_LOGI(TAG, "Baudrate decreased to %lu", current_baudrate);
}

static void handle_uart_baud_get(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    uint32_t br = current_baudrate;
    memcpy(resp, &br, sizeof(br));
    *resp_len = sizeof(br);
    ESP_LOGI(TAG, "Returned baudrate %lu", br);
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
            .maximum_speed = 100000,  // increased to 100kHz
        }
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    ESP_LOGI(TAG, "I2C slave ready (100kHz) @0x%02X", ESP_SLAVE_ADDR);
}

static void uart_init(int baudrate){
    ESP_LOGI(TAG, "Initializing UART with baudrate %d", baudrate);
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized with baudrate %u", baudrate);
}

static void uart_deinit(void) {
    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));
    ESP_LOGI(TAG, "UART deinit");
}

static void uart_read_task(void *arg) {
    (void)arg;
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "UART read task started");
    
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uart_queue_push(data[i]);
            }
            ESP_LOGD(TAG, "Read %d bytes from UART", len);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    
    free(data);
}

static void i2c_slave_task(void *arg) {
    (void)arg;
    uint8_t recv_buf[512];
    uint8_t resp_buf[TRANSFER_BLOCK_SIZE + 16];
    size_t resp_len = 0;
    int consecutive_errors = 0;

    ESP_LOGI(TAG, "I2C slave task started, waiting for commands...");

    for (;;) {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, recv_buf, sizeof(recv_buf),
                                        pdMS_TO_TICKS(100));
        // just for debug
        if (len < 0) {
            consecutive_errors++;
            if (consecutive_errors > 10) {
                ESP_LOGE(TAG, "Too many I2C errors, restarting...");
                consecutive_errors = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            continue;
        }
        
        if (len < 2) {
            continue;
        }
        
        consecutive_errors = 0;  // reset error counter on successful read

        uint16_t cmd = (uint16_t)recv_buf[0] | ((uint16_t)recv_buf[1] << 8);
        
        ESP_LOGI(TAG, "Received command 0x%04X (%d bytes)", cmd, len);
        
        // Log hex dump
        if (len > 2) {
            char hex_str[64] = {0};
            int hex_len = len > 8 ? 8 : len;
            for (int i = 0; i < hex_len; i++) {
                sprintf(hex_str + i*3, "%02X ", recv_buf[i]);
            }
            ESP_LOGD(TAG, "Data: %s%s", hex_str, len > 8 ? "..." : "");
        }
        
        resp_len = 0;
        memset(resp_buf, 0, sizeof(resp_buf));

        handle_command(cmd, recv_buf, (size_t)len, resp_buf, &resp_len);

        if (resp_len > 0) {
            int sent = i2c_slave_write_buffer(I2C_SLAVE_NUM, resp_buf, resp_len,
                                              pdMS_TO_TICKS(300));
            if (sent > 0) {
                ESP_LOGI(TAG, "Sent %d bytes in response to cmd 0x%04X", sent, cmd);
            } else {
                ESP_LOGW(TAG, "Failed to send response to cmd 0x%04X", cmd);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void){

    ESP_LOGI(TAG, "Starting ESP32 I2C slave example, seeking for right baudrate...");
    init_default_baudrate();
    
    i2c_slave_init();
    ESP_LOGI(TAG, "I2C slave initialized, waiting for commands...");
    xTaskCreatePinnedToCore(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "I2C slave task started, ready to receive commands.");
    
    uart_init(current_baudrate);
    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART virtual task started with baudrate %lu", current_baudrate);
    
}
