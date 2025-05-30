#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SLAVE_SCL_IO     15
#define I2C_SLAVE_SDA_IO     23
#define I2C_SLAVE_NUM        I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN 256
#define I2C_SLAVE_RX_BUF_LEN 256
#define ESP_SLAVE_ADDR       0x51

#define PP_API_VERSION       1

#define TRANSFER_BLOCK_SIZE   128
#define SHELL_BUFFER_MAX      256

static const char *TAG = "i2c_slave";

uint8_t uart_app[];
size_t uart_app_len;

typedef enum {
    COMMAND_NONE                  = 0x0000,
    COMMAND_INFO                  = 0x0001,
    COMMAND_APP_INFO              = 0x0002,
    COMMAND_APP_TRANSFER          = 0x0003,
    COMMAND_GETFEATURE_MASK       = 0x0004,
    COMMAND_GETFEAT_DATA_GPS      = 0x0005,
    COMMAND_GETFEAT_DATA_ORIENTATION = 0x0006,
    COMMAND_GETFEAT_DATA_ENVIRONMENT  = 0x0007,
    COMMAND_GETFEAT_DATA_LIGHT    = 0x0008,
    COMMAND_SHELL_PPTOMOD_DATA    = 0x0009,
    COMMAND_SHELL_MODTOPP_DATA_SIZE = 0x000A,
    COMMAND_SHELL_MODTOPP_DATA    = 0x000B,
} Command;

typedef enum { UTILITIES = 0, RX, TX, DEBUG, HOME } app_location_t;

#pragma pack(push,1)
typedef struct {
    uint32_t api_version;
    uint32_t module_version;
    char module_name[20];
    uint32_t application_count;
} device_info;

typedef struct {
    uint32_t header_version;
    uint8_t app_name[16];
    uint8_t bitmap_data[32];
    uint32_t icon_color;
    app_location_t menu_location;
    uint32_t binary_size;
} standalone_app_info;

typedef struct {
    uint16_t angle;
    int16_t  tilt;
} orientation_data_t;
#pragma pack(pop)

static uint8_t shell_buffer[SHELL_BUFFER_MAX];
static size_t  shell_buffer_len = 0;

// ----------------------------------------------------------------
// TODO: Stub function to read application block (e.g., from UART app binary)
// ----------------------------------------------------------------
static size_t read_app_block(uint16_t index, size_t offset, uint8_t *out_buf, size_t max_len) {
    size_t left = uart_app_len - offset;
    size_t to_copy = left < max_len ? left : max_len;
    memcpy(out_buf, uart_app + offset, to_copy);
    return to_copy;
}

void i2c_slave_init() {
    ESP_LOGI(TAG, "Init slave I2C...");
    i2c_config_t conf = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave = {.slave_addr = ESP_SLAVE_ADDR, .maximum_speed = 100000},
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    ESP_LOGI(TAG, "slave I2C init on address 0x%02x", ESP_SLAVE_ADDR);
}

void app_main(void) {
    i2c_slave_init();
    uint8_t data[512];
    uint8_t response[TRANSFER_BLOCK_SIZE + 2];  // +2 for size fields
    size_t resp_len = 0;

    while (true) {
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, sizeof(data), pdMS_TO_TICKS(500));
        if (len < 2) continue;

        uint16_t cmd = data[0] | (data[1] << 8); //decode command (in little-endian)
        resp_len = 0;
        memset(response, 0, sizeof(response));

        switch ((Command)cmd) {

            case COMMAND_INFO: {
                ESP_LOGI(TAG, "Answer with device_info");
                device_info dev = {PP_API_VERSION, 1, "ESP32 Module", 1};
                memcpy(response, &dev, sizeof(dev));
                resp_len = sizeof(dev);
                break;
            }

            // Master requested app info; data[2..3] = index
            case COMMAND_APP_INFO: {
                uint16_t idx = data[2] | (data[3] << 8);
                standalone_app_info app = {1, {0}, {0}, 0x00FF00, DEBUG, 0};
                strncpy((char*)app.app_name, "UART", sizeof(app.app_name));
                memcpy(response, &app, sizeof(app));
                resp_len = sizeof(app);
                break;
            }

            // Master requested block of app; data[2..3]=index, data[4..5]=block
            case COMMAND_APP_TRANSFER: {
                uint16_t idx = data[2] | (data[3] << 8);
                uint16_t blk = data[4] | (data[5] << 8);
                size_t offset = blk * TRANSFER_BLOCK_SIZE;
                resp_len = read_app_block(idx, offset, response, TRANSFER_BLOCK_SIZE);
                break;
            }

            // Report supported features using feature bit positions:
            // bit0=EXT_APP, bit1=UART, bit2=GPS, bit3=ORIENTATION,
            // bit4=ENVIRONMENT, bit5=LIGHT, bit6=DISPLAY, bit7=SHELL
            case COMMAND_GETFEATURE_MASK: {
                uint64_t features = 0;
                features |= (1ULL << 0); // FEAT_EXT_APP
                features |= (1ULL << 1); // FEAT_UART
                features |= (1ULL << 2); // FEAT_GPS
                features |= (1ULL << 3); // FEAT_ORIENTATION
                features |= (1ULL << 4); // FEAT_ENVIRONMENT
                features |= (1ULL << 5); // FEAT_LIGHT
                features |= (1ULL << 6); // FEAT_DISPLAY
                features |= (1ULL << 7); // FEAT_SHELL
                ESP_LOGI(TAG, "Features mask = 0x%016llX", features);
                memcpy(response, &features, sizeof(features));
                resp_len = sizeof(features);
                break;
            }

            case COMMAND_GETFEAT_DATA_GPS: {
                uint8_t gps_data[16] = {0};
                memcpy(response, gps_data, sizeof(gps_data));
                resp_len = sizeof(gps_data);
                break;
            }

            case COMMAND_GETFEAT_DATA_ORIENTATION: {
                orientation_data_t ori = {1234, 5678};  // example values
                ESP_LOGI(TAG, "Sending orientation: angle=%u, tilt=%d", ori.angle, ori.tilt);
                memcpy(response, &ori, sizeof(ori));
                resp_len = sizeof(ori);
                break;
            }

            case COMMAND_GETFEAT_DATA_ENVIRONMENT: {
                struct { float t,h,p; } env = {23.5f, 55.0f, 1013.2f};
                ESP_LOGI(TAG, "Sending environment: T=%.1f, H=%.1f, P=%.1f", env.t, env.h, env.p);
                memcpy(response, &env, sizeof(env));
                resp_len = sizeof(env);
                break;
            }

            case COMMAND_GETFEAT_DATA_LIGHT: {
                uint16_t light = 321;  // example value
                ESP_LOGI(TAG, "Sending light: %u", light);
                memcpy(response, &light, sizeof(light));
                resp_len = sizeof(light);
                break;
            }

            // Receive shell console data from master
            case COMMAND_SHELL_PPTOMOD_DATA: {
                size_t payload = len - 2;
                if (payload > SHELL_BUFFER_MAX) payload = SHELL_BUFFER_MAX;
                memcpy(shell_buffer, &data[2], payload);
                shell_buffer_len = payload;
                //log hex line for received data
                char hexstr[3 * SHELL_BUFFER_MAX + 1] = {0};
                char *h = hexstr;
                for (size_t i = 0; i < payload; i++) {
                    sprintf(h, "%02X ", shell_buffer[i]);
                    h += 3;
                }
                ESP_LOGI(TAG, "Shell RX HEX: %s", hexstr);
                //log printable ASCII line, filter control characters
                char asciistr[SHELL_BUFFER_MAX + 1] = {0};
                size_t ai = 0;
                for (size_t i = 0; i < payload; i++) {
                    uint8_t c = shell_buffer[i];
                    if (c == '\n' || c == '\r') {
                        asciistr[ai++] = c;
                    } else if (isprint(c)) {
                        asciistr[ai++] = c;
                    }
                }
                asciistr[ai] = '>';
                ESP_LOGI(TAG, "Shell RX ASCII: %s", asciistr);
                break;
            }

            // master asking how many shell bytes are available
            case COMMAND_SHELL_MODTOPP_DATA_SIZE: {
                uint16_t size = shell_buffer_len;
                response[0] = size & 0xFF;
                response[1] = (size >> 8) & 0xFF;
                ESP_LOGI(TAG, "Reporting shell size=%u", size);
                resp_len = 2;
                break;
            }

            // master retrieving shell data
            case COMMAND_SHELL_MODTOPP_DATA: {
                uint8_t block = (shell_buffer_len > 64) ? 64 : shell_buffer_len;
                uint8_t flag = (shell_buffer_len > 64) ? 0x80 : 0x00;
                response[0] = flag | (block & 0x7F);
                memcpy(&response[1], shell_buffer, block);
                ESP_LOGI(TAG, "Shell TX ASCII: %.*s", block, shell_buffer);
                resp_len = 1 + block;
                break;
            }

            default:
                ESP_LOGW(TAG, "Unhandled command 0x%04X", cmd);
                break;
        }

        if (resp_len) {
            int sent = i2c_slave_write_buffer(I2C_SLAVE_NUM, response, resp_len, pdMS_TO_TICKS(500));
            ESP_LOGI(TAG, "Sent %d bytes in response to cmd 0x%04X", sent, cmd);
        }
    }
}
