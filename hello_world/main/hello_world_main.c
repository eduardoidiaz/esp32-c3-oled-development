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
#include "store/config/ble_store_config.h"
#include "host/ble_store.h"
#include "host/ble_hs_pvcy.h"  // 🎯 ADD THIS LINE



// Barometer Driver Includes
#include <i2cdev.h>
#include <dps310.h>
#include <mpu6050.h>

// 🛠️ ADD THESE NETWORKING HEADERS FOR ESP-NOW SUPPORT
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"


#include "freertos/timers.h" // Ensure the system timer headers are included

static TimerHandle_t security_watchdog_timer = NULL;

static TimerHandle_t clear_ghost_link_timer = NULL;

static const char *TAG = "OLED_BLE_App";

#define LED_PIN_B 8

// 🎯 TRIPLE-VERIFIED PERFECT PHYSICAL MATRIX GEOMETRY
#define SCREEN_WIDTH  81
#define SCREEN_HEIGHT 40

// 🛠️ FIXING THE DRIVER HEADER BUG EXPLICITLY:
#undef DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa
#define DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa (101325.0f) // True standard sea-level pressure in Pascals


// 128 columns * 64 driver rows / 8 bits = 1024 total bytes allocated safely in RAM
static uint8_t frame_canvas[1024]; 

// Global tracking variables for BLE status updates
static bool ble_connected = false;
static uint16_t conn_handle;

// Global tracking references for the Barometer
static dps310_t dps_sensor_dev;
static mpu6050_dev_t mpu6050_sensor_dev; // Global IMU device profile handle

i2c_master_bus_handle_t bus_handle;

// Shared global data buffers to pass values down into the GATT read pipelines
char counter_string[32]; 

// Storage locations for incoming remote board parameters
static float remote_board_temp = 0.0f;
static float remote_board_press = 0.0f;

// 🛠️ MOVED TO GLOBAL SCOPE SO CALLBACKS CAN ACCESS THEM
float height_change_inches = 0.0f;
esp_lcd_panel_handle_t panel_handle = NULL;

// Storage variables for the remote MPU6050 streaming data
volatile float r_acc_x = 0.0f;
volatile float r_acc_y = 0.0f;
volatile float r_acc_z = 0.0f;
volatile float r_gyro_x = 0.0f;
volatile float r_gyro_y = 0.0f;
volatile float r_gyro_z = 0.0f;


struct __attribute__((packed)) esp_now_payload_t {
    float temp;       // 4 bytes
    float press_hpa;  // 4 bytes
    float acc_x;      // 4 bytes (New IMU variables)
    float acc_y;      // 4 bytes
    float acc_z;      // 4 bytes
    float gyro_x;     // 4 bytes
    float gyro_y;     // 4 bytes
    float gyro_z;     // 4 bytes
}; // 🎯 Absolute Total Size: 32 bytes


static uint8_t slave_mac_address[6] = {0x88, 0x56, 0xA6, 0x2C, 0x09, 0x44};


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

// 🎯 REALIGNED DUAL-BOARD ACCURATE DATA MATRIX INTERFACE
struct __attribute__((packed)) sensor_payload_t {
    float master_temp;  // [0..3 bytes] Master Temperature (°C)
    float master_press; // [4..7 bytes] Master Pressure (hPa)
    float remote_temp;  // [8..11 bytes] Remote Temperature (°C)
    float remote_press; // [12..15 bytes] Remote Pressure (hPa)
    float height_delta; // [16..19 bytes] Filtered Relative Height Change (Inches)
    
    // 🛰️ LOCAL MASTER IMU AXIS MATRIX (Now explicit)
    float master_acc_x; // [20..23 bytes]
    float master_acc_y; // [24..27 bytes]
    float master_acc_z; // [28..31 bytes]
    float master_gyr_x; // [32..35 bytes]
    float master_gyr_y; // [36..39 bytes]
    float master_gyr_z; // [40..43 bytes]

    // 📡 REMOTE SLAVE IMU AXIS MATRIX
    float remote_acc_x; // [44..47 bytes]
    float remote_acc_y; // [48..51 bytes]
    float remote_acc_z; // [52..55 bytes]
    float remote_gyr_x; // [56..59 bytes]
    float remote_gyr_y; // [60..63 bytes]
    float remote_gyr_z; // [64..67 bytes]
}; // 🎯 New Sizing Window: Exactly 68 bytes total payload package


// Global instance to map parameters out to the GATT read table
static struct sensor_payload_t tx_data;




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

void ble_store_config_init(void);
// Forward Declaration for BLE Advertising Trigger
void ble_app_advertise(void);

// 🎯 SECURITY WATCHDOG CALLBACK: Executes if the user ignores the pairing popup!
void security_watchdog_callback(TimerHandle_t xTimer) {
    if (ble_connected) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle, &desc) == 0) {
            // If the connection is STILL unencrypted after 15 seconds, kick them!
            if (!desc.sec_state.encrypted) {
                ESP_LOGE("WATCHDOG", "🛑 Pairing Prompt Timed Out! Forcefully disconnecting iPhone.");
                ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
        }
    }
}

// 🎯 FORCE-KILL CALLBACK: Executes only if a ghost link physically freezes
void force_kill_ghost_link_callback(TimerHandle_t xTimer) {
    // If BLE_GAP_EVENT_DISCONNECT has not fired after 3 full seconds, the stack is stuck
    if (ble_connected) {
        ESP_LOGE("GHOST_RESET", "🚨 CRITICAL: Link frozen after 3 seconds! Forcing low-level stack disconnect.");
        
        // Use standard termination code to clear the stalled handle
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        
        // NOTE: If your stack remains completely locked up here, you can 
        // fall back to a total system reboot via: esp_restart();
    }
}




// 📦 GATT Access Callback Engine
static int gatt_svr_chr_access_custom(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // 🛠️ FIX: Serves raw binary struct memory bytes instead of a text string
        int rc = os_mbuf_append(ctxt->om, &tx_data, sizeof(tx_data));
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
            
            // 🎯 FIXED: Reverted to valid definitions. 
            // In NimBLE, encryption on the link automatically protects the notify channel.
            .flags = BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY, 
                     
            .val_handle = &counter_chr_val_handle,
        }, {
            0, 
        } }
    },
    {
        0, 
    },
};

// 🎯 DYNAMIC RADIO REFRESH HELPER: Forcefully updates advertising data over-the-air
static void ble_app_force_radio_refresh(void) {
    // Stop the active hardware radio channel cleanly
    ble_gap_adv_stop();
    
    // Re-invoke your advertising initializer to recalculate name packets natively
    ble_app_advertise();
    
    ESP_LOGI("RADIO_REFRESH", "⚡ Hardware radio advertising payload refreshed dynamically.");
}

// 📡 BLE Event Management Callback Route
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE Status: Connection Established!");
            if (event->connect.status == 0) {
                ble_connected = true;
                conn_handle = event->connect.conn_handle;
                
                // Actively demand a secure pairing handshake on connect
                ble_gap_security_initiate(conn_handle);

                // START WATCHDOG: Give the user exactly 15 seconds to click "Pair"
                if (security_watchdog_timer == NULL) {
                    security_watchdog_timer = xTimerCreate("ble_sec_watchdog", pdMS_TO_TICKS(15000), 
                                                           pdFALSE, (void *)0, security_watchdog_callback);
                }
                if (security_watchdog_timer != NULL) {
                    xTimerStart(security_watchdog_timer, 0);
                    ESP_LOGI("WATCHDOG", "⏱️ 15-Second Pairing Watchdog Started.");
                }
            } else {
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Status: Connection Terminated!");
            ble_connected = false;
            
            // STOP WATCHDOG: Kill the timer if a disconnection happens naturally
            if (security_watchdog_timer != NULL) {
                xTimerStop(security_watchdog_timer, 0);
            }
            
            // STOP GHOST TIMER: Connection dropped cleanly, no force-kill needed
            if (clear_ghost_link_timer != NULL) {
                xTimerStop(clear_ghost_link_timer, 0);
            }

            // 🎯 FIX: Force an instant over-the-air advertising refresh on every drop.
            // If the name string changed to 'GreenCaddie-Reset' right before the drop,
            // this enforces that the new name is instantly broadcasted to your Swift app!
            ble_app_force_radio_refresh(); 
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete; restarting...");
            ble_app_advertise();
            break;

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            ESP_LOGW(TAG, "⚠️ Phone requested a repeat pairing! Keys are desynced.");
            
            struct ble_gap_conn_desc repeat_desc;
            if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &repeat_desc) == 0) {
                ESP_LOGW(TAG, "🧹 Purging old peer keys immediately to clear the deadlock...");
                
                // Wipe the old keys out of the ESP32 NVS database
                ble_gap_unpair(&repeat_desc.peer_id_addr);
            }
            
            // 🎯 FIX STEP 1: Shift the name string AND force an instant radio refresh!
            // Because repeat pairing doesn't trigger a disconnection event, we must refresh the radio immediately.
            ble_svc_gap_device_name_set("GreenCaddie-Reset");
            ble_app_force_radio_refresh();
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGW("DIAGNOSTIC", "📢 RAW ENCRYPTION CHANGE STATUS CODE: %d", event->enc_change.status);
            
            if (event->enc_change.status == 0) {
                ESP_LOGI("WATCHDOG", "🔒 Handshake Success! Stopping Watchdog Timer.");
                if (security_watchdog_timer != NULL) {
                    xTimerStop(security_watchdog_timer, 0);
                }
                
                // RECOVERY FIX: If the user manually pairs with the device after a reset event, 
                // restore the retail device name string back to production defaults instantly.
                ble_svc_gap_device_name_set("GreenCaddie-Anchor");
            } 
            // Status 5   = Authentication Failure (Often key mismatch)
            // Status 8   = Handshake Canceled (User clicked Cancel)
            // Status 16  = Handshake Timeout (User ignored the popup until it timed out)
            // Status 1288 = User actively clicked Cancel on the iOS Pairing Popup
            else if (event->enc_change.status == 5 || 
                     event->enc_change.status == 8 || 
                     event->enc_change.status == 16 ||
                     event->enc_change.status == 1288) {
                
                ESP_LOGW(TAG, "❌ Security dropped/refused (Status %d). Launching shutdown process.", event->enc_change.status);
                
                if (security_watchdog_timer != NULL) {
                    xTimerStop(security_watchdog_timer, 0);
                }

                // AUTOMATIC BOND CLEANUP
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                    ESP_LOGW(TAG, "🧹 Key mismatch detected. Purging stale peer bond from flash memory...");
                    ble_gap_unpair(&desc.peer_id_addr);
                }
                
                // 🎯 FIX STEP 2: Assign the override name string.
                // The actual radio advertisement payload rebuild will execute natively 
                // inside the BLE_GAP_EVENT_DISCONNECT block right as the link severs!
                ble_svc_gap_device_name_set("GreenCaddie-Reset");
                ESP_LOGW("BRANDING", "🏷️ Shifted name identity string to: GreenCaddie-Reset");
                
                // Attempt a graceful standard software disconnection drop
                int rc = ble_gap_terminate(event->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                
                if (rc == 0) {
                    ESP_LOGI("WATCHDOG", "⏳ Graceful disconnect sent. Arming ghost link backup safety timer.");
                    if (clear_ghost_link_timer == NULL) {
                        clear_ghost_link_timer = xTimerCreate("ghost_killer", pdMS_TO_TICKS(3000), 
                                                              pdFALSE, (void *)0, force_kill_ghost_link_callback);
                    }
                    if (clear_ghost_link_timer != NULL) {
                        xTimerStart(clear_ghost_link_timer, 0);
                    }
                } else {
                    ESP_LOGW(TAG, "Termination request skipped (rc=%d). Stack is already dropping connection.", rc);
                }
            } 
            else if (event->enc_change.status == 13) {
                ESP_LOGI(TAG, "📱 iOS App transitioned to background. Keeping data pipeline active.");
            }
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

    // 🛠️ STEP 1: FORCE PRODUCTION BRANDING OVERRIDE
    // Keep it concise to ensure Packet 2 (Scan Response) fits the 31-byte limit.
    ble_svc_gap_device_name_set("GreenCaddie-Anchor");

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
    
    // Dynamically pulls the "ESP32-C3-REF" string we set above
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
    uint8_t own_addr_type;
    ble_addr_t rnd_addr;
    nvs_handle_t my_nvs_handle;
    esp_err_t err;
    size_t address_size = sizeof(rnd_addr.val);

    // 🎯 THE FINAL ALIGNMENT FIX: Explicitly open the exact "nimble_bond" namespace!
    // This groups your identity MAC seed directly inside the stack's native pairing table.
    err = nvs_open("nimble_bond", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open nimble_bond partition table! err=%d", err);
        return;
    }

    // Try to read an existing permanent MAC address seed out of flash
    err = nvs_get_blob(my_nvs_handle, "ble_mac_seed", rnd_addr.val, &address_size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "💾 No saved BLE identity found in NVS. Generating a permanent identity...");
        int rc = ble_hs_id_gen_rnd(0, &rnd_addr);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to generate random BLE address; rc=%d", rc);
            nvs_close(my_nvs_handle);
            return;
        }

        err = nvs_set_blob(my_nvs_handle, "ble_mac_seed", rnd_addr.val, sizeof(rnd_addr.val));
        if (err == ESP_OK) {
            nvs_commit(my_nvs_handle);
            ESP_LOGI(TAG, "✨ Permanent BLE identity securely locked into nimble_bond flash!");
        }
    } else if (err == ESP_OK) {
        ESP_LOGI(TAG, "✅ Loaded existing permanent BLE identity from nimble_bond partition.");
    }
    nvs_close(my_nvs_handle);

    int rc = ble_hs_id_set_rnd(rnd_addr.val);
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

void init_imu(void) {
    // Zero out the whole structural block cleanly
    memset(&mpu6050_sensor_dev, 0, sizeof(mpu6050_dev_t));

    // 🎯 1. BIND DESCRIPTOR WITH YOUR PINS (Handle, Addr, Port, SDA, SCL)
    esp_err_t err = mpu6050_init_desc(&mpu6050_sensor_dev, 0x68, 0, 5, 6);
    if (err != ESP_OK) {
        ESP_LOGE("IMU_ERR", "Descriptor allocation dropped: %s", esp_err_to_name(err));
        return;
    }

    // 🎯 2. INITIALIZE HARDWARE CORE TRANSCEIVER INTERRUPT STATES
    err = mpu6050_init(&mpu6050_sensor_dev);
    if (err != ESP_OK) {
        ESP_LOGE("IMU_ERR", "MPU6050 core failed to wake up: %s", esp_err_to_name(err));
        return;
    }

    // 🎯 3. SECURE FIXED TELEMETRY GAIN TARGET RANGES NATIVELY
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&mpu6050_sensor_dev, MPU6050_GYRO_RANGE_250));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&mpu6050_sensor_dev, MPU6050_ACCEL_RANGE_2));

    ESP_LOGI("IMU_OK", "Slave MPU6050 IMU Bound & Operational!");
}

void init_barometer(void) {
    // 1. Initialize the baseline i2cdev helper library layer
    ESP_ERROR_CHECK(i2cdev_init());

    // 2. Clear descriptions safely out of tracking RAM layout spaces
    memset(&dps_sensor_dev, 0, sizeof(dps310_t));

    // Initialize config macro to seed internal driver structures safely
    dps310_config_t dps_config = DPS310_CONFIG_DEFAULT();
    dps_config.tmp_oversampling = DPS310_PM_PRC_128; // Keep high for resolution
    dps_config.pm_oversampling = DPS310_PM_PRC_128;
    dps_config.tmp_rate = DPS310_TMP_RATE_1;
    dps_config.pm_rate = DPS310_PM_RATE_64;
    dps_config.tmp_src = 1; // Keep your fix to pull from the internal temperature diode

    // 3. Initialize the physical pin connection mappings
    ESP_ERROR_CHECK(dps310_init_desc(&dps_sensor_dev, DPS310_I2C_ADDRESS_1, I2C_NUM_0, 5, 6));

    // Force internal software pullups active
    dps_sensor_dev.i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    dps_sensor_dev.i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;

    // 4. Initialize device framework contexts
    ESP_ERROR_CHECK(dps310_init(&dps_sensor_dev, &dps_config));

    // 🛠️ NEW: Wait for the sensor and internal coefficients matrices to become ready on the bus
    bool sensor_ready = false;
    bool coef_ready = false;
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!sensor_ready) {
            dps310_is_ready_for_sensor(&dps_sensor_dev, &sensor_ready);
        }
        if (!coef_ready) {
            dps310_is_ready_for_coef(&dps_sensor_dev, &coef_ready);
        }
    } while (!sensor_ready || !coef_ready);

    // 🛠️ NEW: Extract and apply factory-fused internal calculation coefficients
    ESP_ERROR_CHECK(dps310_get_coef(&dps_sensor_dev));

    // 🛠️ NEW: Calibrate absolute altitude baseline lookup metrics to 18.288 meters
    ESP_ERROR_CHECK(dps310_calibrate_altitude(&dps_sensor_dev, 18.288f));

    // 🛠️ NEW: Spin up the background tracking pipeline engines using the proper API function
    ESP_ERROR_CHECK(dps310_backgorund_start(&dps_sensor_dev, DPS310_MODE_BACKGROUND_ALL));

    ESP_LOGI(TAG, "DPS310 Barometer Initialized & Calibrated Successfully!");
}

void esp_now_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {

    // SECURE GATEWAY CHECK: Reject any packets that don't come from our specific Slave
    if (recv_info == NULL || memcmp(recv_info->src_addr, slave_mac_address, 6) != 0) {
        // 📝 FIXED PURE-C LOG: Manually prints the 6-byte hex elements to bypass macro errors
        if (recv_info != NULL) {
            ESP_LOGD("SECURITY_GUARD", "Ignored rogue ESP-NOW frame from source MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                     recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        }
        return; 
    }

    esp_lcd_panel_handle_t local_display = panel_handle; // Map global screen context safely

    // 🎯 GATING TARGET DETECTS EXPANED 32-BYTE PAYLOAD SIZE AUTOMATICALLY
    if (len == sizeof(struct esp_now_payload_t)) {
        struct esp_now_payload_t *incoming = (struct esp_now_payload_t *)data;
        remote_board_temp = incoming->temp;
        remote_board_press = incoming->press_hpa;

        // Fetch fresh Master metrics immediately when the packet arrives
        float m_temp = 0.0f, m_press = 0.0f, m_alt = 0.0f, m_calc_alt = 0.0f;
        dps310_read_temp(&dps_sensor_dev, &m_temp);
        dps310_read_pressure(&dps_sensor_dev, &m_press);
        dps310_read_altitude(&dps_sensor_dev, &m_alt);
        dps310_calc_altitude(&dps_sensor_dev, m_press, &m_calc_alt);

        float m_press_hpa = m_press / 100.0f;

        mpu6050_raw_acceleration_t acce_raw = {0, 0, 0};
        mpu6050_raw_rotation_t gyro_raw = {0, 0, 0};

        if (mpu6050_get_raw_acceleration(&mpu6050_sensor_dev, &acce_raw) != ESP_OK ||
            mpu6050_get_raw_rotation(&mpu6050_sensor_dev, &gyro_raw) != ESP_OK) {
            ESP_LOGW("LOCAL_IMU", "Failed to sample master local IMU over I2C.");
        }

        static float dual_board_baseline_offset = -999.0f;
        
        // 🛠️ LOW-PASS FILTER VARIABLES:
        static float filtered_height_inches = 0.0f;
        const float alpha = 0.15f; // 0.15 means 15% new data, 85% old smooth data

        // Calculate the raw directional altitude delta (Remote - Master)
        float remote_calculated_altitude_meters = 0.0f;
        dps310_calc_altitude(&dps_sensor_dev, remote_board_press * 100.0f, &remote_calculated_altitude_meters);
        
        float raw_height_inches = (remote_calculated_altitude_meters - m_calc_alt) * 39.3701f;

        // Lock in your ground-zero calibration offset layout on the very first packet
        if (dual_board_baseline_offset == -999.0f) {
            dual_board_baseline_offset = raw_height_inches;
            filtered_height_inches = 0.0f; // Seed the filter initial state
        }

        // Apply baseline calibration tare calibration
        float clean_unfiltered_inches = raw_height_inches - dual_board_baseline_offset;
        
        // THE 1-LINE LOW-PASS EMA FILTER IMPLEMENTATION:
        filtered_height_inches = (alpha * clean_unfiltered_inches) + ((1.0f - alpha) * filtered_height_inches);
        
        // 🎯 CONVERT NEGATIVE DESK NOISE DRIFT TO ZERO:
        if (filtered_height_inches < 0.0f) {
            height_change_inches = 0.0f; 
        } else if (fabsf(clean_unfiltered_inches) < 0.3f) {
            height_change_inches = filtered_height_inches;
        } else {
            height_change_inches = filtered_height_inches;
        }

        // 🎯 1. DIRECT REALIGNED PACKET VALUE ASSIGNMENTS:
        tx_data.master_temp  = m_temp;
        tx_data.master_press = m_press_hpa;
        tx_data.remote_temp  = remote_board_temp;
        tx_data.remote_press = remote_board_press;
        tx_data.height_delta = height_change_inches; // Low-pass filtered inches delta [0x1.22]

        // 🎯 2. MAP LOCAL HARDWARE CHANNELS (Converts to 'g' and '°/s' for iOS) [0x1.23]
        tx_data.master_acc_x = (float)acce_raw.x / 16384.0f;
        tx_data.master_acc_y = (float)acce_raw.y / 16384.0f;
        tx_data.master_acc_z = (float)acce_raw.z / 16384.0f;
        tx_data.master_gyr_x = (float)gyro_raw.x / 131.0f;
        tx_data.master_gyr_y = (float)gyro_raw.y / 131.0f;
        tx_data.master_gyr_z = (float)gyro_raw.z / 131.0f;

        // 🎯 3. MAP REMOTE OVER-THE-AIR CHANNELS (Converts to 'g' and '°/s' for iOS) [0x1.23]
        tx_data.remote_acc_x = incoming->acc_x / 16384.0f;
        tx_data.remote_acc_y = incoming->acc_y / 16384.0f;
        tx_data.remote_acc_z = incoming->acc_z / 16384.0f;
        tx_data.remote_gyr_x = incoming->gyro_x / 131.0f;
        tx_data.remote_gyr_y = incoming->gyro_y / 131.0f;
        tx_data.remote_gyr_z = incoming->gyro_z / 131.0f;

        // Enhanced telemetry matrix log output confirming your new format structure [0x1.23]
        ESP_LOGI("SYNC_ALT","Data Stream Bundled! Size: %d bytes. Shipping to iPhone notification buffer.", 
                 (int)sizeof(tx_data));

        // 🛠️ ACTIVE SECURITY LOCK: (Keeps your encrypted NimBLE notification layer rolling seamlessly) [0x1.23]
        if (ble_connected) {
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(conn_handle, &desc) == 0) {
                if (desc.sec_state.encrypted) {
                    // 🚀 The sizeof dynamic expression here automatically shifts to notify the iPhone of all 68 bytes!
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(&tx_data, sizeof(tx_data));
                    if (om != NULL) {
                        ble_gattc_notify_custom(conn_handle, counter_chr_val_handle, om);
                    }
                } else {
                    ESP_LOGW("SECURITY_GUARD", "⚠️ Connection alive but unencrypted. Data blocked!");
                }
            }
        }


        // SCALED UNIFIED TELEMETRY LOG ENGINE (WITH METRIC UNITS)
        // ESP_LOGI("SYNC_ALT", 
        //          "--- FULL SCALED TELEMETRY MATRIX ---\n"
        //          "BARO: M_Temp: %.2f C | M_Pres: %.2f hPa\n"
        //          "BARO: R_Temp: %.2f C | R_Pres: %.2f hPa\n"
        //          "CALC: Filtered Height: %.2f in\n"
        //          "LOCAL  IMU: Accel: [%.2fg, %.2fg, %.2fg] | Gyro: [%.1f°/s, %.1f°/s, %.1f°/s]\n"
        //          "REMOTE IMU: Accel: [%.2fg, %.2fg, %.2fg] | Gyro: [%.1f°/s, %.1f°/s, %.1f°/s]",
        //          m_temp, m_press_hpa,
        //          remote_board_temp, remote_board_press,
        //          height_change_inches,
                 
        //          // 🛰️ Local Master Hardware Conversion Calculations (Raw / LSB Sensitivity) [0x1.23]
        //          (float)acce_raw.x / 16384.0f, (float)acce_raw.y / 16384.0f, (float)acce_raw.z / 16384.0f,
        //          (float)gyro_raw.x / 131.0f,   (float)gyro_raw.y / 131.0f,   (float)gyro_raw.z / 131.0f,
                 
        //          // 📡 Remote Slave Over-the-Air Conversion Calculations [0x1.23]
        //          incoming->acc_x / 16384.0f, incoming->acc_y / 16384.0f, incoming->acc_z / 16384.0f,
        //          incoming->gyro_x / 131.0f,   incoming->gyro_y / 131.0f,   incoming->gyro_z / 131.0f);

        // Format and refresh your OLED display canvas text frames
        char t_buf[32], p_buf[32], h_buf[32], a_buf[32];
        snprintf(t_buf, sizeof(t_buf), "TEMP: %.1f C", m_temp);
        snprintf(p_buf, sizeof(p_buf), "PRES: %.1f hPa", m_press_hpa);
        snprintf(h_buf, sizeof(h_buf), "HGHT: %.1f in", height_change_inches);
        // snprintf(a_buf, sizeof(a_buf), "ACCZ: %.0f", tx_data.sensor_z);

        memset(frame_canvas, 0, sizeof(frame_canvas));
        draw_string(0, 2, t_buf);
        draw_string(0, 18, p_buf);
        draw_string(0, 34, h_buf);
        draw_string(0, 50, a_buf); // Added live Z-acceleration value display line

        if (local_display != NULL) {
            esp_lcd_panel_draw_bitmap(local_display, 0, 0, 128, 64, frame_canvas);
        }
    } else {
         ESP_LOGW("ESPNOW_ERR", "Packet Size mismatch! Expected 32, got %d", len);
    }
}




void init_master_esp_now(void) {
    // 1. Initialize the global network interface system layers
    ESP_ERROR_CHECK(esp_netif_init());

    // This must remain to prevent internal Wi-Fi driver task crashes
    ESP_ERROR_CHECK(esp_event_loop_create_default()); 
    
    // 2. Load basic Wi-Fi structural configuration presets
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 3. Force storage to local RAM instead of NVS to prevent read/write bottlenecks
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // 4. Force Station Mode (STA) so the internal radio layout matches raw ESP-NOW parameters
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // 5. Spin up the physical radio antenna system layers
    ESP_ERROR_CHECK(esp_wifi_start());

    // Lock the radio frequency channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    // 6. Initialize the core peer-to-peer ESP-NOW message transmission engines
    ESP_ERROR_CHECK(esp_now_init());
    
    // 7. Attach your real-time data frame collector callback routing function
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_callback));
    
    // 🎯 ADD THIS PEER REGISTRY BLOCK TO THE MASTER:
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, slave_mac_address, 6);
    peer_info.channel = 1;         // Must match Channel 1
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA; // Explicitly map to Master's Station interface
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    
    ESP_LOGI("RADIO_INIT", "🎯 Target Unicast Link with Slave Board Secured!");
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

    // --- 2. INITIALIZE RAM DATA STRUCTURE MEMORY FIELDS ---
    // 🎯 Seeding buffers ensures early incoming packets do not process trash memory locations
    memset(&tx_data, 0, sizeof(tx_data));
    snprintf(counter_string, sizeof(counter_string), "T:0.0C P:0.0hPa H:0.0in");

    // --- 3. INITIALIZE THE DPS310 BAROMETER SENSOR CORE ---
    init_barometer();
    init_imu();
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- 4. INITIALIZE THE PHYSICAL OLED SCREEN PANEL ---
    esp_lcd_panel_io_handle_t io_handle = NULL;
    panel_handle = NULL; // Assign directly into your global tracker variable

    i2c_master_bus_handle_t actual_bus_handle = NULL;
    ESP_ERROR_CHECK(i2cdev_get_shared_handle(I2C_NUM_0, (void **)&actual_bus_handle));

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,               
        .scl_speed_hz = 400000,          
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,      
        .lcd_param_bits = 8,    
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(actual_bus_handle, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1, 
        .bits_per_pixel = 1,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_set_gap(panel_handle, 0, 0); 
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // --- 5. INITIALIZE DIAGNOSTIC LED PIN ---
    gpio_reset_pin(LED_PIN_B);
    gpio_set_direction(LED_PIN_B, GPIO_MODE_OUTPUT);
    int led_state = 0;

    // Clear frame buffer layout spaces natively
    memset(frame_canvas, 0x00, sizeof(frame_canvas));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 128, 64, frame_canvas));
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- 6. INITIALIZE COEXISTENCE RADIO STEP A: WIRELESS ESP-NOW FIRST ---
    // This safely reserves the underlying Wi-Fi MAC stacks before BLE spins up
    init_master_esp_now();
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow internal radio allocation layers to settle

        // --- 7. INITIALIZE COEXISTENCE RADIO STEP B: BLE STACK SUBSYSTEM ---
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NimBLE stack; error=%d", ret);
        return;
    }
    
    ble_gatts_reset();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // ENABLE SECURITY AND BONDING FOR AUTO-SAVING
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; // Keep your display configuration choice
    ble_hs_cfg.sm_bonding = 1;                  // 1 = Enable Bonding (Saves the device!)
    ble_hs_cfg.sm_mitm = 0;                     // MITM protection not required for this link
    ble_hs_cfg.sm_sc = 1;                       // Enable Secure Connections (LE Secure Connections)

    // 🎯 FIX: Map the encryption and identity keys to the correct structural members
    // This tells the stack to distribute and save both the LTK and the IRK privacy keys!
    ble_hs_cfg.sm_our_key_dist |= (BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    ble_hs_cfg.sm_their_key_dist |= (BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

   // Populate service configuration limits
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    // 🎯 CRITICAL FIX: Bind callbacks and register the storage config handlers 
    // immediately AFTER adding your services, but BEFORE launching the host task loop.
    ble_hs_cfg.store_read_cb = ble_store_config_read;
    ble_hs_cfg.store_write_cb = ble_store_config_write;
    
    // Maps the internal security bonds backend directly to the "nvs" partition
    ble_store_config_init();

    // Bind sync callback
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // Spawn background task thread layer to handle Bluetooth advertising callbacks
    xTaskCreate(ble_host_task, "ble_host_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Starting Master Heartbeat Thread Lane...");

    // --- 8. PRIMARY SYSTEM HEARTBEAT THREAD LANE ---
    while (1) {
        // Toggle the physical status LED to verify that the microcontroller remains active
        gpio_set_level(LED_PIN_B, led_state);
        led_state = !led_state;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


