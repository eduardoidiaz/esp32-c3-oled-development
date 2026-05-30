#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h" 
#include "driver/i2c_master.h"     
#include "esp_lcd_panel_io.h"     
#include "esp_lcd_panel_vendor.h" 
#include "esp_lcd_panel_ops.h"    
#include "esp_lcd_panel_ssd1306.h" 
#include "esp_log.h"   

// BLE Core NimBLE Library Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "nvs_flash.h"

#include "services/gatt/ble_svc_gatt.h"

// 🎯 GLOBAL STATIC 128-BIT UUID INSTANCES
// static const ble_uuid128_t custom_svc_uuid = {
//     .u.type = BLE_UUID_TYPE_128,
//     .value = {0xab, 0x89, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd,
//               0xab, 0x89, 0x67, 0x45, 0x23, 0x01, 0xcd, 0xab}
// };

// static const ble_uuid128_t custom_chr_uuid = {
//     .u.type = BLE_UUID_TYPE_128,
//     .value = {0xac, 0x89, 0x67, 0x45, 0x23, 0x01, 0xef, 0xcd,
//               0xab, 0x89, 0x67, 0x45, 0x23, 0x01, 0xcd, 0xab}
// };

// 🎯 TRANSLATED ZEPHYR TO NIMBLE UUID CORES
// Service: 00001523-1212-efde-1523-785feabcd123
static const ble_uuid128_t custom_svc_uuid = {
    .u.type = BLE_UUID_TYPE_128,
    .value = {0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
              0xde, 0xef, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00}
};

// Characteristic: 00001524-1212-efde-1523-785feabcd123
static const ble_uuid128_t custom_chr_uuid = {
    .u.type = BLE_UUID_TYPE_128,
    .value = {0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15,
              0xde, 0xef, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00}
};


// Global reference handle to push notifications down to the phone
static uint16_t counter_chr_val_handle;



static const char *TAG = "OLED_BLE_App";

#define LED_PIN_B 8

// 🎯 TRIPLE-VERIFIED PERFECT PHYSICAL MATRIX GEOMETRY
#define SCREEN_WIDTH  81
#define SCREEN_HEIGHT 40

// 128 columns * 64 driver rows / 8 bits = 1024 total bytes allocated safely in RAM
static uint8_t frame_canvas[1024]; 

// Global tracking variables for BLE status updates
static bool ble_connected = false;
static uint16_t conn_handle;

// 🛠️ THE FIX: Declare counter_string as a global variable here so GATT can find it
char counter_string[32]; 

// Expanded 8x8 ASCII Monochrome Font Dictionary Array (Characters 32 to 122)
static const uint8_t font8x8_basic[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // Space (32)
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x6C,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00}, // "
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // #
    {0x18,0x7E,0xC0,0x7C,0x06,0xFC,0x18,0x00}, // $
    {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC3,0x00}, // %
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // &
    {0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00}, // '
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // (
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // ,
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // .
    {0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x00}, // /
    {0x3E,0x61,0x67,0x6D,0x79,0x43,0x3E,0x00}, // 0
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 1
    {0x3E,0x66,0x06,0x1E,0x30,0x66,0x7E,0x00}, // 2
    {0x3E,0x66,0x06,0x1C,0x06,0x66,0x3E,0x00}, // 3
    {0x06,0x0E,0x1E,0x36,0x7F,0x06,0x06,0x00}, // 4
    {0x7F,0x60,0x7E,0x06,0x06,0x66,0x3E,0x00}, // 5
    {0x3E,0x66,0x60,0x7E,0x66,0x66,0x3E,0x00}, // 6
    {0x7F,0x06,0x0C,0x18,0x30,0x30,0x30,0x00}, // 7
    {0x3E,0x66,0x66,0x3E,0x66,0x66,0x3E,0x00}, // 8
    {0x3E,0x66,0x66,0x3F,0x06,0x66,0x3E,0x00}, // 9
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // :
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30}, // ;
    {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, // <
    {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00}, // =
    {0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00}, // >
    {0x3E,0x66,0x06,0x0C,0x18,0x00,0x18,0x00}, // ?
    {0x3E,0x66,0x6F,0x6D,0x6D,0x60,0x3E,0x00}, // @
    {0x18,0x3C,0x66,0x66,0x7F,0x66,0x66,0x00}, // A
    {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00}, // B
    {0x3E,0x66,0x60,0x60,0x60,0x66,0x3E,0x00}, // C
    {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00}, // D
    {0x7F,0x60,0x60,0x7C,0x60,0x60,0x7F,0x00}, // E
    {0x7F,0x60,0x60,0x7C,0x60,0x60,0x60,0x00}, // F
    {0x3E,0x66,0x60,0x6C,0x66,0x66,0x3A,0x00}, // G
    {0x66,0x66,0x66,0x7F,0x66,0x66,0x66,0x00}, // H
    {0x7E,0x18,0x18,0x18,0x18,0x18,0x7E,0x00}, // I
    {0x1E,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // J
    {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00}, // K
    {0x60,0x60,0x60,0x60,0x60,0x60,0x7F,0x00}, // L
    {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00}, // M
    {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00}, // N
    {0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x00}, // O
    {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // P
    {0x3E,0x66,0x66,0x66,0x6D,0x67,0x3E,0x0F}, // Q
    {0x7C,0x66,0x66,0x7C,0x6C,0x66,0x66,0x00}, // R
    {0x3E,0x66,0x60,0x3E,0x06,0x66,0x3E,0x00}, // S
    {0x7F,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // T
    {0x66,0x66,0x66,0x66,0x66,0x66,0x3E,0x00}, // U
    {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00}, // X
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00}, // Y
    {0x7F,0x06,0x0C,0x18,0x30,0x60,0x7F,0x00}, // Z
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // [ (91)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // \ (92)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ] (93)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ^ (94)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // _ (95)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ` (96)
    {0x00,0x00,0x3C,0x06,0x3E,0x66,0x3B,0x00}, // a (97)
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x7C,0x00}, // b
    {0x00,0x00,0x3E,0x60,0x60,0x66,0x3E,0x00}, // c
    {0x06,0x06,0x3E,0x66,0x66,0x66,0x3B,0x00}, // d
    {0x00,0x00,0x3E,0x66,0x7E,0x60,0x3E,0x00}, // e
    {0x1C,0x30,0x78,0x30,0x30,0x30,0x78,0x00}, // f
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x3C}, // g
    {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x00}, // h
    {0x18,0x00,0x38,0x18,0x18,0x18,0x3C,0x00}, // i
    {0x06,0x00,0x0E,0x06,0x06,0x66,0x3C,0x00}, // j
    {0x60,0x60,0x6C,0x78,0x70,0x6C,0x66,0x00}, // k
    {0x38,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // l
    {0x00,0x00,0x66,0x7F,0x6B,0x63,0x63,0x00}, // m
    {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00}, // n
    {0x00,0x00,0x3E,0x66,0x66,0x66,0x3E,0x00}, // o
    {0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60}, // p
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x06}, // q
    {0x00,0x00,0x6E,0x70,0x60,0x60,0x60,0x00}, // r
    {0x00,0x00,0x3E,0x60,0x3E,0x06,0x7C,0x00}, // s
    {0x18,0x18,0x7E,0x18,0x18,0x18,0x0D,0x03}, // t
    {0x00,0x00,0x66,0x66,0x66,0x66,0x3B,0x00}, // u
    {0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00}, // v
    {0x00,0x00,0x63,0x6B,0x7F,0x3E,0x14,0x00}, // w
    {0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00}, // x
    {0x00,0x00,0x66,0x66,0x66,0x3E,0x06,0x3C}, // y
    {0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00}  // z
};

void draw_pixel(int x, int y, bool color) {
    if (x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT) return;
    int real_x = x + 20; 
    if (real_x < 0 || real_x >= 128) return;
    
    int index = real_x + (y / 8) * 128;
    if (index < 0 || index >= sizeof(frame_canvas)) return;

    if (color) {
        frame_canvas[index] |= (1 << (y % 8));   
    } else {
        frame_canvas[index] &= ~(1 << (y % 8));  
    }
}

void draw_string(int x, int y, const char *str) {
    while (*str) {
        char c = *str;
        if (c >= 32 && c <= 122) {
            int font_index = c - 32;
            for (int row = 0; row < 8; row++) {
                uint8_t byte = font8x8_basic[font_index][row];
                for (int col = 0; col < 8; col++) {
                    if (byte & (1 << (7 - col))) {
                        draw_pixel(x + col, y + row, true);
                    }
                }
            }
        }
        x += 8; 
        str++;
    }
}

// 📦 GATT Access Callback Engine
static int gatt_svr_chr_access_custom(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // If the phone requests a Read operation, feed it your counter string data
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, counter_string, strlen(counter_string));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// 🏢 GATT Service & Characteristic Definition Layout Matrix
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &custom_svc_uuid.u, // 🛠️ FIX: Passes memory reference pointer safely
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &custom_chr_uuid.u, // 🛠️ FIX: Passes memory reference pointer safely
            .access_cb = gatt_svr_chr_access_custom,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, 
            .val_handle = &counter_chr_val_handle,
        }, {
            0, // Marks the termination boundary of this specific characteristic subarray
        } }
    },
    {
        0, // Marks the termination boundary of the main service list array matrix
    },
};




// Forward Declaration for BLE Advertising Trigger
void ble_app_advertise(void);

// 📡 BLE Event Management Callback Route
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE Status: Connection Established!");
            if (event->connect.status == 0) {
                ble_connected = true;
                conn_handle = event->connect.conn_handle;
            } else {
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Status: Connection Terminated!");
            ble_connected = false;
            ble_app_advertise(); // Restart beacon transmissions instantly
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete; restarting...");
            ble_app_advertise();
            break;

        default:
            break;
    }
    return 0;
}

// 📡 FIXED SPLIT-PACKET ADVERTISEMENT ROUTINE (Fits the 31-byte limit)
void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields adv_fields;
    struct ble_hs_adv_fields rsp_fields;
    const char *device_name;
    uint8_t own_addr_type;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address configuration type; rc=%d", rc);
        return;
    }

    // --- PACKET 1: Main Advertisement Payload (Contains flags & custom UUID) ---
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // Inject the custom Service UUID your Swift app is looking for
    adv_fields.uuids128 = (ble_uuid128_t *)&custom_svc_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting main advertisement fields; rc=%d", rc);
        return;
    }

    // --- PACKET 2: Scan Response Payload (Contains the device name string) ---
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    device_name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    // Map the name data array to the secondary hardware scanning channel lanes
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response fields; rc=%d", rc);
        return;
    }

    // Launch connection parameters
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertisement operations; rc=%d", rc);
    }
}



void ble_app_on_sync(void) {
    uint8_t own_addr_type; // Correctly matching primitive byte type configuration
    
    // Pass the memory pointer down to the auto-inference allocator engine
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
        return;
    }
    
    // Begin broadcasting immediately upon driver stabilization
    ble_app_advertise();
}


void ble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Thread Initialized.");
    nimble_port_run(); // Blocks execution natively
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    // 🛠️ THE FIX: Initialize Non-Volatile Storage (NVS) for the BLE Controller
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- 1. OLED Display Layout Hardware Initialization ---
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 6,
        .sda_io_num = 5,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_config, &bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x3C,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t global_i2c_dev_handle;
    i2c_master_bus_add_device(bus_handle, &dev_config, &global_i2c_dev_handle);

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,                 
        .scl_speed_hz = 400000,           
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_new_panel_io_i2c(bus_handle, &io_config, &io_handle);

    esp_lcd_panel_ssd1306_config_t ssd1306_vendor_cfg = { .height = 64 };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,             
        .bits_per_pixel = 1,              
        .vendor_config = &ssd1306_vendor_cfg, 
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle);
    
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_set_gap(panel_handle, 0, 0); 
    // esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // --- 2. BLE Stack Subsystem Initialization ---
    // 🛠️ ADD THIS DELAY: Let the hardware RF calibration settle completely first
    vTaskDelay(pdMS_TO_TICKS(200));

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE stack; error=%d", ret);
        return;
    }

    ble_svc_gap_device_name_set("ESP32C3-OLED");

    
    // 🛠️ THE GATT SERVER FIX: Reset and map our custom database table components
    ble_gatts_reset();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Feed the custom service definitions down to the system routing cache
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    xTaskCreate(ble_host_task, "ble_host_task", 4096, NULL, 5, NULL);


    // --- 3. Pin & Local Buffer Configuration Loop Settings ---
    gpio_reset_pin(LED_PIN_B);
    gpio_set_direction(LED_PIN_B, GPIO_MODE_OUTPUT);
    int led_state = 0;

    int counter = 0;

    memset(frame_canvas, 0x00, sizeof(frame_canvas));
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 127, 63, frame_canvas);
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {
        gpio_set_level(LED_PIN_B, led_state);
        led_state = !led_state;

        memset(frame_canvas, 0x00, sizeof(frame_canvas));

        draw_string(8, 0, "ESP32-C3");

        if (ble_connected) {
            draw_string(6, 10, "BLE: Online");
            draw_string(6, 20, "Connected");
        } else {
            draw_string(6, 10, "BLE: Ready");
            draw_string(6, 20, "Beaconing");
        }

        snprintf(counter_string, sizeof(counter_string), "Count: %d", counter++);
        draw_string(4, 30, counter_string);

        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 127, 63, frame_canvas);

        // 🚀 LIVE GATT NOTIFICATION: If a phone is connected, stream the data straight to it!
        if (ble_connected) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(counter_string, strlen(counter_string));
            if (om != NULL) {
                ble_gatts_notify_custom(conn_handle, counter_chr_val_handle, om);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
