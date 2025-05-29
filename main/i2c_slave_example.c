#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_SLAVE_SCL_IO          15
#define I2C_SLAVE_SDA_IO          23
#define I2C_SLAVE_NUM             I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN      128
#define I2C_SLAVE_RX_BUF_LEN      128
#define ESP_SLAVE_ADDR            0x51

#define PP_API_VERSION            1

typedef enum {
    COMMAND_NONE = 0,
    COMMAND_INFO = 1,
    COMMAND_APP_INFO,
    COMMAND_APP_TRANSFER,
    COMMAND_GETFEATURE_MASK,
    COMMAND_GETFEAT_DATA_GPS,
    COMMAND_GETFEAT_DATA_ORIENTATION,
    COMMAND_GETFEAT_DATA_ENVIRONMENT,
    COMMAND_GETFEAT_DATA_LIGHT,
    COMMAND_SHELL_PPTOMOD_DATA,
    COMMAND_SHELL_MODTOPP_DATA_SIZE,
    COMMAND_SHELL_MODTOPP_DATA,
} Command;

static const char *TAG = "i2c_slave";

typedef enum {
    UTILITIES = 0,
    RX,
    TX,
    DEBUG,
    HOME
} app_location_t;

typedef struct {
    uint32_t api_version;
    uint32_t module_version;
    char module_name[20];
    uint32_t application_count;
} __attribute__((packed)) device_info;

typedef struct {
    uint32_t header_version;
    uint8_t app_name[16];
    uint8_t bitmap_data[32];
    uint32_t icon_color;
    app_location_t menu_location;
    uint32_t binary_size;
} __attribute__((packed)) standalone_app_info;

// Custom command handler typedef
typedef void (*custom_handler_t)(const uint8_t *in_data, size_t in_len, uint8_t *out_data, size_t *out_len);

// Entry struct
typedef struct {
    uint16_t command;
    custom_handler_t handler;
} custom_command_entry_t;

// MAX custom commands
#define MAX_CUSTOM_COMMANDS 10

// Static list of custom commands
static custom_command_entry_t custom_commands[MAX_CUSTOM_COMMANDS];
static int custom_command_count = 0;

// Register a command
void register_custom_command(uint16_t cmd, custom_handler_t handler) {
    if (custom_command_count < MAX_CUSTOM_COMMANDS) {
        custom_commands[custom_command_count].command = cmd;
        custom_commands[custom_command_count].handler = handler;
        custom_command_count++;
    } else {
        ESP_LOGW(TAG, "Cannot register command 0x%04X, list full", cmd);
    }
}

// Example handler for 0x007F
void handler_007F(const uint8_t *in_data, size_t in_len, uint8_t *out_data, size_t *out_len) {
    ESP_LOGI(TAG, "Handling custom command 0x007F");
    const char *msg = "Hello from 0x007F!";
    size_t len = strlen(msg);
    memcpy(out_data, msg, len);
    *out_len = len;
}

void i2c_slave_init() {
    ESP_LOGI(TAG, "Init slave I2C...");

    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave = {
            .slave_addr = ESP_SLAVE_ADDR,
            .maximum_speed = 100000
        }
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &conf_slave));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE,
                                       I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
    ESP_LOGI(TAG, "slave I2C init on address 0x%02x", ESP_SLAVE_ADDR);
}

void app_main(void) {
    i2c_slave_init();

    // Register custom command 0x007F
    register_custom_command(0x007F, handler_007F);

    uint8_t data[128];
    uint8_t response[128];
    int resp_len = 0;

    while (1) {
        ESP_LOGI(TAG, "Waiting for master...");
        int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, sizeof(data), pdMS_TO_TICKS(1000));

        if (len >= 2) {
            uint16_t cmd = (data[1] << 8) | data[0];
            ESP_LOGI(TAG, "Command received: 0x%04X", cmd);
            resp_len = 0;
            memset(response, 0, sizeof(response));

            switch ((Command)cmd) {
                case COMMAND_INFO: {
                    ESP_LOGI(TAG, "Answer with device_info");
                    device_info dev = {
                        .api_version = PP_API_VERSION,
                        .module_version = 0x00010000,
                        .application_count = 1
                    };
                    strncpy(dev.module_name, "ESP32 Module", sizeof(dev.module_name));
                    memcpy(response, &dev, sizeof(device_info));
                    resp_len = sizeof(device_info);
                    break;
                }

                case COMMAND_APP_INFO: {
                    ESP_LOGI(TAG, "Answer with app_info");
                    standalone_app_info app = {
                        .header_version = 1,
                        .icon_color = 0x00FF00,
                        .menu_location = DEBUG,
                        .binary_size = 0
                    };
                    strncpy((char*)app.app_name, "UART", sizeof(app.app_name));
                    memset(app.bitmap_data, 0xFF, sizeof(app.bitmap_data));
                    memcpy(response, &app, sizeof(app));
                    resp_len = sizeof(app);
                    break;
                }

                case COMMAND_GETFEATURE_MASK: {
                    ESP_LOGI(TAG, "Answer with mask of supported features");
                    uint64_t features = (1ULL << 1);
                    memcpy(response, &features, sizeof(features));
                    resp_len = sizeof(features);
                    break;
                }

                case COMMAND_SHELL_MODTOPP_DATA_SIZE: {
                    ESP_LOGI(TAG, "Answer with shell data size");
                    uint16_t size = 19;
                    memcpy(response, &size, sizeof(size));
                    resp_len = sizeof(size);
                    break;
                }

                case COMMAND_SHELL_PPTOMOD_DATA: {
                    ESP_LOGI(TAG, "Received shell data");
                    if (len > 2) {
                        printf("Shell RX: ");
                        for (int i = 0; i < len - 2; i++) {
                            printf("%c", data[2 + i]);
                        }
                        printf("\n");
                    }
                    break;
                }

                case COMMAND_SHELL_MODTOPP_DATA: {
                    ESP_LOGI(TAG, "Sending shell response");
                    const char *msg = "Hola desde ESP32 :)";
                    size_t msg_len = strlen(msg);
                    response[0] = msg_len & 0x3F;
                    memcpy(&response[1], msg, msg_len);
                    resp_len = msg_len + 1;
                    break;
                }

                case COMMAND_GETFEAT_DATA_ENVIRONMENT: {
                    ESP_LOGI(TAG, "Mock environment data");
                    struct {
                        float temperature;
                        float humidity;
                        float pressure;
                    } __attribute__((packed)) env = {23.5, 55.0, 1013.2};
                    memcpy(response, &env, sizeof(env));
                    resp_len = sizeof(env);
                    break;
                }

                default: {
                    int handled = 0;
                    for (int i = 0; i < custom_command_count; i++) {
                        if (custom_commands[i].command == cmd) {
                            custom_commands[i].handler(data + 2, len - 2, response, (size_t *)&resp_len);
                            handled = 1;
                            break;
                        }
                    }
                    if (!handled) {
                        ESP_LOGW(TAG, "Unhandled command: 0x%04X", cmd);
                    }
                    break;
                }
            }

            if (resp_len > 0) {
                ESP_LOGI(TAG, "Sending answer (%d bytes)", resp_len);
                int sent = i2c_slave_write_buffer(I2C_SLAVE_NUM, response, resp_len, pdMS_TO_TICKS(1000));
                if (sent <= 0) {
                    ESP_LOGW(TAG, "FAIL when sending answer");
                }
            }
        } else {
            ESP_LOGW(TAG, "Invalid data received, len: %d", len);
        }
    }
}
