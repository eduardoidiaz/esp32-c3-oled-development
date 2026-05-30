#include <stdio.h>
#include <string.h>
#include <math.h>
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

// Barometer Driver Includes
#include <i2cdev.h>
#include <dps310.h>

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

// Global tracking references for the Barometer
static dps310_t dps_sensor_dev;

i2c_master_bus_handle_t bus_handle;

// Shared global data buffers to pass values down into the GATT read pipelines
char counter_string[32]; 

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
        .uuid = &custom_svc_uuid.u, 
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &custom_chr_uuid.u, 
            .access_cb = gatt_svr_chr_access_custom,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, 
            .val_handle = &counter_chr_val_handle,
        }, {
            0, 
        } }
    },
    {
        0, 
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
            ble_app_advertise(); 
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

// 📡 SPLIT-PACKET ADVERTISEMENT ROUTINE (Fits the 31-byte limit)
void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields adv_fields;
    struct ble_hs_adv_fields rsp_fields;
    const char *device_name;
    uint8_t own_addr_type;
    int rc;

    rc = ble_hs_id_infer_auto(1, &own_addr_type); // Checks for Random profiles
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer dynamic address profile; rc=%d", rc);
        return;
    }

    // --- PACKET 1: Main Advertisement Payload (Flags + Custom UUID) ---
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = (ble_uuid128_t *)&custom_svc_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting main advertisement fields; rc=%d", rc);
        return;
    }

    // --- PACKET 2: Scan Response Payload (Custom Name String) ---
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    device_name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response fields; rc=%d", rc);
        return;
    }

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
    ble_addr_t rnd_addr;    
    uint8_t own_addr_type;
    
    int rc = ble_hs_id_gen_rnd(0, &rnd_addr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to generate random BLE address; rc=%d", rc);
        return;
    }
    
    rc = ble_hs_id_set_rnd(rnd_addr.val);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(1, &own_addr_type);
    assert(rc == 0);

    ble_app_advertise();
}

void ble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Thread Initialized.");
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

void init_barometer(void) {
    // 1. Initialize the baseline i2cdev helper library layer
    ESP_ERROR_CHECK(i2cdev_init());

    // 2. Clear descriptions safely out of tracking RAM layout spaces
    memset(&dps_sensor_dev, 0, sizeof(dps310_t));

    // 3. 🛠️ RESTORE THIS LINE: Target the single hardware I2C_NUM_0 on shared pins 5 and 6
    ESP_ERROR_CHECK(dps310_init_desc(&dps_sensor_dev, DPS310_I2C_ADDRESS_1, I2C_NUM_0, 5, 6));

    // 👇 ADD THESE TWO LINES HERE TO ENABLE INTERNAL PULLUPS:
    dps_sensor_dev.i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dps_sensor_dev.i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;

    // 4. Construct your standard default performance configurations
    dps310_config_t dps_config;
    memset(&dps_config, 0, sizeof(dps310_config_t));
    
    dps_config.pm_rate = DPS310_PM_RATE_32;         
    dps_config.tmp_rate = DPS310_TMP_RATE_32;       
    dps_config.pm_oversampling = DPS310_PM_PRC_8;  
    dps_config.tmp_oversampling = DPS310_TMP_PRC_8; 
    dps_config.tmp_src = 1;
    ESP_LOGI(TAG, "dps_config.tmp_src = %d", dps_config.tmp_src);

    // 5. Pass device structural pointers into driver initialization logic
    esp_err_t rc = dps310_init(&dps_sensor_dev, &dps_config);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize barometer hardware core; error=%d", rc);
        return;
    }

    // Place the device into background measurement tracking mode natively
    ESP_ERROR_CHECK(dps310_set_mode(&dps_sensor_dev, DPS310_MODE_BACKGROUND_ALL));

    ESP_LOGI(TAG, "DPS310 Barometer Initialized Successfully!");
}



void app_main(void)
{
    // --- 1. INITIALIZE SYSTEM FLASH MEMORY ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- 2. INITIALIZE THE DPS310 BAROMETER CORE FIRST ---
    // This allows i2cdev to cleanly claim the single hardware Port 0 (SDA=5, SCL=6)
    init_barometer();
    vTaskDelay(pdMS_TO_TICKS(100)); // Let the hardware settle

    // 1. Correctly declare BOTH handles at the top of the block
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL; // Fixed: No longer undeclared!

    // 2. Retrieve the thread-safe I2C handle using the official framework function
    i2c_master_bus_handle_t actual_bus_handle = NULL;
    esp_err_t handle_err = i2cdev_get_shared_handle(I2C_NUM_0, (void **)&actual_bus_handle); // Fixed typo

    if (handle_err != ESP_OK || actual_bus_handle == NULL) {
        ESP_LOGE("OLED_INIT", "Could not retrieve the shared i2cdev bus handle!");
        return;
    }

    // 3. Your updated, modern bit-length panel IO configuration
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,               
        .scl_speed_hz = 400000,          
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,      
        .lcd_param_bits = 8,    
    };

    // 4. Attach the OLED IO interface to the shared master bus handle
    esp_err_t io_err = esp_lcd_new_panel_io_i2c(actual_bus_handle, &io_config, &io_handle);
    if (io_err != ESP_OK) {
        ESP_LOGE("OLED_INIT", "Failed to create I2C IO handle: %s", esp_err_to_name(io_err));
        return; 
    }

    // 5. Configure the physical SSD1306 display panel properties
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1, // Change to your reset physical pin number if you use one
        .bits_per_pixel = 1,
    };

    // 6. This will now compile cleanly on Line 421
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    
    
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_set_gap(panel_handle, 0, 0); 
    // esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // --- 3. INITIALIZE THE DPS310 SENSOR CONTROLLER ---
    // Small delay allows physical I2C pins to settle between configurations
    vTaskDelay(pdMS_TO_TICKS(100)); 
    init_barometer();

    // --- 4. INITIALIZE BLE STACK SUBSYSTEM ---
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE stack; error=%d", ret);
        return;
    }

    ble_svc_gap_device_name_set("ESP32C3-OLED");
    
    ble_gatts_reset();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // Fire up background task execution thread lane
    xTaskCreate(ble_host_task, "ble_host_task", 4096, NULL, 5, NULL);

    // --- 5. INITIALIZE DIAGNOSTIC LED PIN ---
    gpio_reset_pin(LED_PIN_B);
    gpio_set_direction(LED_PIN_B, GPIO_MODE_OUTPUT);
    int led_state = 0;

    int counter = 0;
    char sensor_string[32];

    // Initial silicon memory blackout sweep
    memset(frame_canvas, 0x00, sizeof(frame_canvas));
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 127, 63, frame_canvas);
    vTaskDelay(pdMS_TO_TICKS(100));

        // --- 4. DATA READING, LOW-PASS FILTER, & INCH ALTITUDE LOOP ---
    float temperature = 0.0;
    float pressure_pa = 0.0;
    float current_hpa = 0.0;
    
    // Low-Pass Filter tracking variables
    float filtered_hpa = -1.0f;
    const float alpha = 0.05f; 

    // Baseline and Altitude tracking variables
    float baseline_hpa = -1.0f; 
    float relative_delta_hpa = 0.0f;
    float height_change_inches = 0.0f;

    char oled_temp_buf[32];
    char oled_press_buf[32];

    ESP_LOGI(TAG, "Starting Low-Pass Filtered Inch Altimeter Loop...");

    while (1) {
        esp_err_t res_t = dps310_read_temp(&dps_sensor_dev, &temperature);
        esp_err_t res_p = dps310_read_pressure(&dps_sensor_dev, &pressure_pa);

        if (res_t == ESP_OK && res_p == ESP_OK) {
            current_hpa = pressure_pa / 100.0f;

            // Initialize the filter with the first raw reading on boot
            if (filtered_hpa < 0.0f) {
                filtered_hpa = current_hpa;
                baseline_hpa = current_hpa;
                ESP_LOGI(TAG, "🎯 Baseline & Filter Seeded: %.3f hPa", baseline_hpa);
            }

            // Apply the low-pass exponential filter
            filtered_hpa = (alpha * current_hpa) + ((1.0f - alpha) * filtered_hpa);

            // Calculate hPa difference from your baseline desk height
            relative_delta_hpa = fabsf(filtered_hpa - baseline_hpa);

            // 📐 CONVERT hPa TO INCHES:
            // 1 hPa ≈ 329.1 feet at room temp. 329.1 feet * 12 inches = 3949.2 inches
            height_change_inches = relative_delta_hpa * 3949.2f;

            // Update your phone's BLE string buffer layout
            snprintf(counter_string, sizeof(counter_string), "H: %.1f in", height_change_inches);
            
            // 🛠️ UPDATED WITH YOUR EXACT FORMAT PLUSSING ADDITIONAL INCH COLUMNS
            ESP_LOGI(TAG, "RAW: %.3f | FILTERED: %.3f | Change: %.3f hPa | Height: %.2f in", 
                     current_hpa, filtered_hpa, relative_delta_hpa, height_change_inches);

            // Format strings for your physical screen layout matrix
            snprintf(oled_temp_buf, sizeof(oled_temp_buf),   "TEMP: %.1f C", temperature);
            snprintf(oled_press_buf, sizeof(oled_press_buf), "HGHT: %.1f in", height_change_inches);

            // 1. Clear the old RAM canvas buffer entirely 
            memset(frame_canvas, 0, sizeof(frame_canvas));

            // 2. Render text onto the virtual matrix layout maps
            draw_string(0, 8, oled_temp_buf);
            draw_string(0, 24, oled_press_buf);

            // 3. Flush the tracking data matrix arrays to physical display pins
            ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 128, 64, frame_canvas));

        } else {
            ESP_LOGE(TAG, "I2C Error! Temp: %s | Press: %s", esp_err_to_name(res_t), esp_err_to_name(res_p));
        }

        // Poll every 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }






    /*
    // --- 6. PRIMARY RUNTIME PROCESS LOOP ENGINE ---
    while (1) {
        // Toggle the onboard onboard LED to show the code is alive
        gpio_set_level(LED_PIN_B, led_state);
        led_state = !led_state;

        // Reset display memory layout matrix space to pure black
        memset(frame_canvas, 0x00, sizeof(frame_canvas));

        // Read data coordinates from the weather barometer
        float temperature = 0.0f;
        float pressure = 0.0f; 
        bool ready = false;

        // 🛠️ ALTERNATIVE 4-ARGUMENT SIGNATURE FIX (Only use if dps310_is_ready isn't found)
        // Checks the MEAS_CFG register (0x08) using the sensor-ready flag bit mask (0x70)
        if (dps310_is_ready_for(&dps_sensor_dev, 0x08, 0x70, &ready) == ESP_OK && ready) {
            // 🛠️ THE COMPILER FIX: Use separate, explicit readout functions 
            // This bypasses unexported header structures completely!
            dps310_read_pressure(&dps_sensor_dev, &pressure);
            dps310_read_temp(&dps_sensor_dev, &temperature);
        }


        // Draw application tag lines onto the buffer screen
        draw_string(8, 0, "Esp32-c3");

        // Format and render live temperature numbers
        snprintf(sensor_string, sizeof(sensor_string), "Temp: %.1f C", temperature);
        draw_string(4, 10, sensor_string);

        // Format and render live pressure numbers (Converted to hPa)
        snprintf(sensor_string, sizeof(sensor_string), "Pres: %.1f hPa", pressure / 100.0f);
        draw_string(4, 20, sensor_string);

        // Track wireless communication connection states
        if (ble_connected) {
            snprintf(counter_string, sizeof(counter_string), "Connected %d", counter++);
            draw_string(4, 30, counter_string);
            
            // PUSH LIVE TELEMETRY PAYLOAD TO IPHONE APP
            // We transmit the raw text data currently sitting inside counter_string
            struct os_mbuf *om = ble_hs_mbuf_from_flat(counter_string, strlen(counter_string));
            if (om != NULL) {
                ble_gatts_notify_custom(conn_handle, counter_chr_val_handle, om);
            }
        } else {
            snprintf(counter_string, sizeof(counter_string), "Beaconing %d", counter++);
            draw_string(4, 30, counter_string);
        }

        // Push layout buffer modifications down to hardware channels
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 127, 63, frame_canvas);

        // Frame cycle delay constraint (1 second interval)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    */
}
