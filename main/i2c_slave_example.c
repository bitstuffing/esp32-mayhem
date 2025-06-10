#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

// TTGO T-Beam v1.1 Pin Configuration
#define I2C_SLAVE_SCL_IO_TTGO        22
#define I2C_SLAVE_SDA_IO_TTGO        21
#define I2C_SLAVE_SCL_IO        15
#define I2C_SLAVE_SDA_IO        23
#define I2C_SLAVE_NUM           I2C_NUM_0
#define I2C_SLAVE_TX_BUF_LEN    256
#define I2C_SLAVE_RX_BUF_LEN    256
#define ESP_SLAVE_ADDR          0x51

// LoRa SX1276/SX1278 pins
#define LORA_SCK_PIN            5   // SCK
#define LORA_MISO_PIN           19  // MISO
#define LORA_MOSI_PIN           27  // MOSI
#define LORA_CS_PIN             18  // CS
#define LORA_RST_PIN            14  // RST
#define LORA_DIO0_PIN           26  // DIO0
#define LORA_DIO1_PIN           33  // DIO1 
#define LORA_DIO2_PIN           32  // DIO2

// UART (for LoRa communication)
#define UART_TX_PIN             17  // TX LoRa
#define UART_RX_PIN             16  // RX LoRa
// NEO-6M/NEO-8M GPS pins
#define GPS_TX_PIN              34  // GPS TX (ESP32 RX)
#define GPS_RX_PIN              12  // GPS RX (ESP32 TX)

// OLED SSD1306 
#define OLED_SDA_PIN            21  // I2C_SLAVE_SDA_IO
#define OLED_SCL_PIN            22  // I2C_SLAVE_SCL_IO
#define OLED_ADDR               0x3C // I2C address OLED
#define OLED_WIDTH              128
#define OLED_HEIGHT             64

// OLED Commands
#define OLED_CMD_SET_CONTRAST       0x81
#define OLED_CMD_DISPLAY_RAM        0xA4
#define OLED_CMD_DISPLAY_ALLON      0xA5
#define OLED_CMD_DISPLAY_NORMAL     0xA6
#define OLED_CMD_DISPLAY_INVERTED   0xA7
#define OLED_CMD_DISPLAY_OFF        0xAE
#define OLED_CMD_DISPLAY_ON         0xAF
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3
#define OLED_CMD_SET_COM_PINS       0xDA
#define OLED_CMD_SET_VCOM_DETECT    0xDB
#define OLED_CMD_SET_DISPLAY_CLK_DIV 0xD5
#define OLED_CMD_SET_PRECHARGE      0xD9
#define OLED_CMD_SET_MULTIPLEX      0xA8
#define OLED_CMD_SET_LOW_COLUMN     0x00
#define OLED_CMD_SET_HIGH_COLUMN    0x10
#define OLED_CMD_SET_START_LINE     0x40
#define OLED_CMD_MEMORY_MODE        0x20
#define OLED_CMD_COLUMN_ADDR        0x21
#define OLED_CMD_PAGE_ADDR          0x22
#define OLED_CMD_COM_SCAN_DEC       0xC8
#define OLED_CMD_SEG_REMAP          0xA1
#define OLED_CMD_CHARGE_PUMP        0x8D
#define OLED_CMD_EXTERNAL_VCC       0x1
#define OLED_CMD_SWITCH_CAP_VCC     0x2

// Simple 5x8 font for OLED
static const uint8_t font5x8[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // space
    {0x00, 0x00, 0x5f, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1c, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1c, 0x00}, // )
    {0x14, 0x08, 0x3e, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3e, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3e}, // @
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // A
    {0x7f, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7f, 0x41, 0x41, 0x22, 0x1c}, // D
    {0x7f, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3e, 0x41, 0x49, 0x49, 0x7a}, // G
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // H
    {0x00, 0x41, 0x7f, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3f, 0x01}, // J
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7f, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // M
    {0x7f, 0x04, 0x08, 0x10, 0x7f}, // N
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // O
    {0x7f, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // Q
    {0x7f, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // T
    {0x3f, 0x40, 0x40, 0x40, 0x3f}, // U
    {0x1f, 0x20, 0x40, 0x20, 0x1f}, // V
    {0x3f, 0x40, 0x38, 0x40, 0x3f}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

// LED and Button pins
#define LED_PIN                 4   // LED T-Beam v1.1
#define BUTTON_PIN              38  // button T-Beam v1.1

#define PP_API_VERSION          1
#define TRANSFER_BLOCK_SIZE     128
#define SHELL_BUFFER_MAX        256
#define GPS_UART_BUF_SIZE       256
#define GPS_UART_PORT_NUM       UART_NUM_1 // GPS on UART1
#define GPS_UART_BAUDRATE       9600
#define MAX_APPLICATIONS        1

static const char *TAG = "i2c_slave";

//TODO, replace with external UART app
// This is a simulated UART application for testing purposes.
// In a real application, this would be replaced with the actual UART app binary.
static uint8_t uart_app[] = {
    // Header PortaPack App 
    0x50, 0x50, 0x41, 0x50,  // "PPAP" - PortaPack App signature
    0x01, 0x00, 0x00, 0x00,  // Version 1
    0x4C, 0x4F, 0x52, 0x41,  // "LORA"
    0x00, 0x00, 0x00, 0x00,  // Padding
    
    // Simulated binary code for the LoRa app
    0xE9, 0x00, 0x00, 0x00,  // Jump instruction
    0x4C, 0x6F, 0x52, 0x61,  // "LoRa"
    0x20, 0x54, 0x54, 0x47,  // " TTG"
    0x4F, 0x20, 0x38, 0x36,  // "O 86"
    0x38, 0x4D, 0x48, 0x7A,  // "8MHz"
    0x00, 0x00, 0x00, 0x00,  // Padding
    
    // more dummy instructions
    0x90, 0x90, 0x90, 0x90,  // NOPs
    0xC3, 0x00, 0x00, 0x00,  // RET instruction
    
    // fill the rest of the app with zeroes until the block size
    [36 ... TRANSFER_BLOCK_SIZE - 1] = 0x00
};

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
    // LoRa specific commands (TODO)
    COMMAND_LORA_FREQUENCY_SET     = 0x7F06,
    COMMAND_LORA_FREQUENCY_GET     = 0x7F07,
    COMMAND_LORA_POWER_SET         = 0x7F08,
    COMMAND_LORA_POWER_GET         = 0x7F09,
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

// GPS Data Structure

typedef struct
{
    uint8_t hour;      /*!< Hour */
    uint8_t minute;    /*!< Minute */
    uint8_t second;    /*!< Second */
    uint16_t thousand; /*!< Thousand */
} ppgps_time_t;

typedef struct
{
    uint8_t day;   /*!< Day (start from 1) */
    uint8_t month; /*!< Month (start from 1) */
    uint16_t year; /*!< Year (start from 2000) */
} ppgps_date_t;

typedef struct
{
    float latitude;       /*!< Latitude (degrees) */
    float longitude;      /*!< Longitude (degrees) */
    float altitude;       /*!< Altitude (meters) */
    uint8_t sats_in_use;  /*!< Number of satellites in use */
    uint8_t sats_in_view; /*!< Number of satellites in view */
    float speed;          /*!< Ground speed, unit: m/s */
    ppgps_date_t date;    /*!< Fix date */
    ppgps_time_t tim;     /*!< time in UTC */
} ppgpssmall_t;

typedef struct
{
    float angle;
    float tilt;
} orientation_t;

static ppgpssmall_t current_gps_data = {0};

static uint8_t shell_buffer[SHELL_BUFFER_MAX];
static size_t shell_buffer_len = 0;

#define UART_PORT_NUM      UART_NUM_2 // use UART2 for LoRa communication
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

static uint32_t lora_frequency = 868000000; // 868MHz by default
static uint8_t lora_power = 14; // 14dBm by default

static void i2c_slave_init(void);
static void uart_init(int baudrate);
static void uart_deinit(void);
static void uart_read_task(void *arg);
static void lora_gpio_init(void);
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

static void lora_gpio_init(void) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0); // LED off
    
    gpio_set_direction(LORA_SCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_MOSI_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_MISO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LORA_DIO0_PIN, GPIO_MODE_INPUT);
    
    gpio_set_level(LORA_CS_PIN, 1);    // CS up
    gpio_set_level(LORA_RST_PIN, 1);   // RST up
    gpio_set_level(LORA_SCK_PIN, 0);   // SCK down
    gpio_set_level(LORA_MOSI_PIN, 0);  // MOSI down
    
    ESP_LOGI(TAG, "LoRa GPIO initialized");
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

static void handle_lora_frequency_set(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    if (req_len < 6) { // 2 bytes for command + 4 bytes from frequency
        ESP_LOGW(TAG, "LORA_FREQUENCY_SET: insufficient data");
        *resp_len = 0;
        return;
    }
    
    uint32_t freq = (uint32_t)req[2] | ((uint32_t)req[3] << 8) | 
                    ((uint32_t)req[4] << 16) | ((uint32_t)req[5] << 24);
    
    // 868MHz range validation
    if (freq >= 863000000 && freq <= 870000000) {
        lora_frequency = freq;
        ESP_LOGI(TAG, "LoRa frequency set to %lu Hz", lora_frequency);
        resp[0] = 0x00; // OK
    } else {
        ESP_LOGW(TAG, "Invalid LoRa frequency: %lu Hz", freq);
        resp[0] = 0xFF; // Error
    }
    *resp_len = 1;
}

static void handle_lora_frequency_get(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    memcpy(resp, &lora_frequency, sizeof(lora_frequency));
    *resp_len = sizeof(lora_frequency);
    ESP_LOGI(TAG, "Returned LoRa frequency %lu Hz", lora_frequency);
}

static void handle_lora_power_set(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    if (req_len < 3) { // 2 bytes for command + 1 byte for power
        ESP_LOGW(TAG, "LORA_POWER_SET: insufficient data");
        *resp_len = 0;
        return;
    }
    
    uint8_t power = req[2];
    
    // validate range (0-20 dBm for SX1276)
    if (power <= 20) {
        lora_power = power;
        ESP_LOGI(TAG, "LoRa power set to %u dBm", lora_power);
        resp[0] = 0x00; // OK
    } else {
        ESP_LOGW(TAG, "Invalid LoRa power: %u dBm", power);
        resp[0] = 0xFF; // Error
    }
    *resp_len = 1;
}

static void handle_lora_power_get(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    resp[0] = lora_power;
    *resp_len = 1;
    ESP_LOGI(TAG, "Returned LoRa power %u dBm", lora_power);
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
    strncpy((char*)app.app_name, "UART - LoRa", sizeof(app.app_name));
    
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

static void handle_getfeat_data_orientation(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    (void)req; (void)req_len;
    orientation_t ori = { .angle = 12.5f, .tilt = 56.3f };
    memcpy(resp, &ori, sizeof(ori));
    *resp_len = sizeof(ori);
    ESP_LOGI(TAG, "Sent orientation data (angle=%.1f, tilt=%.1f)", ori.angle, ori.tilt);
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
    
    const int max_data_length = 4;
    size_t available = uart_queue_available();
    size_t to_send = available < max_data_length ? available : max_data_length;
    bool more_data = available > max_data_length;
    
    resp[0] = (to_send & 0x7F) | (more_data ? 0x80 : 0x00);
    
    for (int i = to_send; i < max_data_length; i++) {
        resp[i + 1] = 0xFF;
    }
    
    for (size_t i = 0; i < to_send; i++) {
        resp[i + 1] = uart_queue_pop();
    }
    
    *resp_len = 1 + max_data_length; 
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

static spi_device_handle_t spi_lora;

static void lora_spi_init(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_MISO_PIN,
        .mosi_io_num = LORA_MOSI_PIN,
        .sclk_io_num = LORA_SCK_PIN,
        .quadwp_io_num = -1, //TODO
        .quadhd_io_num = -1, //TODO
        .max_transfer_sz = 32,
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1MHz
        .mode = 0,
        .spics_io_num = LORA_CS_PIN,
        .queue_size = 7,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_lora));
    
    ESP_LOGI(TAG, "LoRa SPI initialized");
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

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 initialized with baudrate %u on pins TX:%d RX:%d", baudrate, UART_TX_PIN, UART_RX_PIN);
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
// GPS NMEA Parser
static void parse_nmea_sentence(const char* sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char sentence_copy[256];
        strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
        sentence_copy[sizeof(sentence_copy) - 1] = '\0';
        
        char *token = strtok(sentence_copy, ",");
        int field = 0;
        
        while (token != NULL && field < 15) {
            switch (field) {
                case 1: // Time HHMMSS.SSS
                    if (strlen(token) >= 6) {
                        char hour_str[3] = {token[0], token[1], '\0'};
                        char min_str[3] = {token[2], token[3], '\0'};
                        char sec_str[3] = {token[4], token[5], '\0'};
                        
                        current_gps_data.tim.hour = atoi(hour_str);
                        current_gps_data.tim.minute = atoi(min_str);
                        current_gps_data.tim.second = atoi(sec_str);
                        
                        if (strlen(token) > 7 && token[6] == '.') {
                            char ms_str[4] = {0};
                            strncpy(ms_str, &token[7], 3);
                            current_gps_data.tim.thousand = atoi(ms_str);
                        }
                    }
                    break;
                case 2: // Latitude DDMM.MMMM
                    if (strlen(token) > 0) {
                        float lat = atof(token);
                        current_gps_data.latitude = (int)(lat/100) + (lat - (int)(lat/100)*100)/60.0f;
                    }
                    break;
                case 3: // Latitude N/S
                    if (strlen(token) > 0 && token[0] == 'S') {
                        current_gps_data.latitude = -current_gps_data.latitude;
                    }
                    break;
                case 4: // Longitude DDDMM.MMMM
                    if (strlen(token) > 0) {
                        float lon = atof(token);
                        current_gps_data.longitude = (int)(lon/100) + (lon - (int)(lon/100)*100)/60.0f;
                    }
                    break;
                case 5: // Longitude E/W
                    if (strlen(token) > 0 && token[0] == 'W') {
                        current_gps_data.longitude = -current_gps_data.longitude;
                    }
                    break;
                case 6: // Fix quality (0=invalid, 1=GPS fix, 2=DGPS fix)
                    current_gps_data.sats_in_use = (atoi(token) > 0) ? 1 : 0;
                    break;
                case 7: // Number of satellites
                    current_gps_data.sats_in_view = atoi(token);
                    break;
                case 9: // Altitude
                    current_gps_data.altitude = atof(token);
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
    }
    // parse RMC to get speed and date
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char sentence_copy[256];
        strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
        sentence_copy[sizeof(sentence_copy) - 1] = '\0';
        
        char *token = strtok(sentence_copy, ",");
        int field = 0;
        
        while (token != NULL && field < 12) {
            switch (field) {
                case 7: // Speed in knots
                    if (strlen(token) > 0) {
                        float speed_knots = atof(token);
                        current_gps_data.speed = speed_knots * 0.514444f; // Convert to m/s
                    }
                    break;
                case 9: // Date DDMMYY
                    if (strlen(token) == 6) {
                        char day_str[3] = {token[0], token[1], '\0'};
                        char month_str[3] = {token[2], token[3], '\0'};
                        char year_str[3] = {token[4], token[5], '\0'};
                        
                        current_gps_data.date.day = atoi(day_str);
                        current_gps_data.date.month = atoi(month_str);
                        current_gps_data.date.year = 2000 + atoi(year_str);
                    }
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
    }
}

// GPS UART Task
static void gps_uart_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(GPS_UART_BUF_SIZE);
    char nmea_buffer[256] = {0};
    int nmea_index = 0;
    
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for GPS UART buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "GPS UART task started");
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT_NUM, data, GPS_UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                if (c == '\n' || c == '\r') {
                    if (nmea_index > 0) {
                        nmea_buffer[nmea_index] = '\0';
                        parse_nmea_sentence(nmea_buffer);
                        nmea_index = 0;
                    }
                } else if (nmea_index < sizeof(nmea_buffer) - 1) {
                    nmea_buffer[nmea_index++] = c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
}

// Initialize GPS UART
static void gps_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT_NUM, GPS_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT_NUM, GPS_RX_PIN, GPS_TX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "GPS UART initialized on pins TX:%d RX:%d", GPS_RX_PIN, GPS_TX_PIN);
}

// Updated GPS data handler
static void handle_getfeat_data_gps(uint8_t *req, size_t req_len, uint8_t *resp, size_t *resp_len) {
    ESP_LOGI(TAG, "Handling GPS data request");
    (void)req; (void)req_len;
    
    // Pack real GPS data using PortaPack structure
    memcpy(resp, &current_gps_data, sizeof(ppgpssmall_t));
    *resp_len = sizeof(ppgpssmall_t);
    
    ESP_LOGI(TAG, "Sent GPS data: Lat:%.6f Lon:%.6f Alt:%.2f Speed:%.2f Sats_use:%d Sats_view:%d Date:%02d/%02d/%04d Time:%02d:%02d:%02d.%03d", 
             current_gps_data.latitude, current_gps_data.longitude, 
             current_gps_data.altitude, current_gps_data.speed,
             current_gps_data.sats_in_use, current_gps_data.sats_in_view,
             current_gps_data.date.day, current_gps_data.date.month, current_gps_data.date.year,
             current_gps_data.tim.hour, current_gps_data.tim.minute, current_gps_data.tim.second, current_gps_data.tim.thousand);
}

// OLED Display functions 
static void oled_init(void) {
    //TODO: Implementar
    ESP_LOGI(TAG, "OLED SSD1306 initialized on I2C address 0x%02X", OLED_ADDR);
}

static void display_status(void) {
    // TODO: Implementar
    ESP_LOGD(TAG, "Display updated - GPS: Lat:%.6f Lon:%.6f Sats_use:%d Sats_view:%d", 
             current_gps_data.latitude, current_gps_data.longitude,
             current_gps_data.sats_in_use, current_gps_data.sats_in_view);
}

static void tbeam_gpio_init(void) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0); // LED off initially
    
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
    
    // LoRa GPIO setup
    gpio_set_direction(LORA_SCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_MOSI_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_MISO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LORA_DIO0_PIN, GPIO_MODE_INPUT);
    
    gpio_set_level(LORA_CS_PIN, 1);    // CS high
    gpio_set_level(LORA_RST_PIN, 1);   // RST high
    gpio_set_level(LORA_SCK_PIN, 0);   // SCK low
    gpio_set_level(LORA_MOSI_PIN, 0);  // MOSI low
    
    ESP_LOGI(TAG, "T-Beam GPIO initialized");
}

static void display_task(void *arg) {
    (void)arg;
    
    while (1) {
        display_status();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
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
            case COMMAND_LORA_FREQUENCY_SET:
                handle_lora_frequency_set(req_buf, req_len, resp_buf, resp_len);
                break;
            case COMMAND_LORA_FREQUENCY_GET:
                handle_lora_frequency_get(req_buf, req_len, resp_buf, resp_len);
                break;
            case COMMAND_LORA_POWER_SET:
                handle_lora_power_set(req_buf, req_len, resp_buf, resp_len);
                break;
            case COMMAND_LORA_POWER_GET:
                handle_lora_power_get(req_buf, req_len, resp_buf, resp_len);
                break;
            default:
                ESP_LOGW(TAG, "Unhandled command 0x%04X", cmd);
                break;
        }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting TTGO T-Beam v1.1 I2C slave...");
    
    tbeam_gpio_init();
    init_default_baudrate();
    
    i2c_slave_init();
    oled_init();
    
    ESP_LOGI(TAG, "I2C slave initialized, waiting for commands...");
    xTaskCreatePinnedToCore(i2c_slave_task, "i2c_slave_task", 4096, NULL, 10, NULL, 1);
    ESP_LOGI(TAG, "I2C slave task started, ready to receive commands.");
    
    gps_uart_init();
    xTaskCreate(gps_uart_task, "gps_uart_task", 4096, NULL, 9, NULL);
    ESP_LOGI(TAG, "GPS UART task started");
    
    uart_init(current_baudrate);
    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 8, NULL);
    ESP_LOGI(TAG, "LoRa UART task started with baudrate %lu", current_baudrate);
    
    xTaskCreate(display_task, "display_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Display task started");
    
    gpio_set_level(LED_PIN, 1);
    ESP_LOGI(TAG, "TTGO T-Beam v1.1 system ready!");
}
