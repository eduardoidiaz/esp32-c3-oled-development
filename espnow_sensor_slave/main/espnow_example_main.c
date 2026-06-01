#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include <i2cdev.h>
#include <dps310.h>
#include <mpu6050.h> // Ensure your component include path matches your mpu6050 library layout


// 🛠️ FIXING THE DRIVER HEADER BUG ON THE SLAVE EXPLICITLY:
#undef DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa
#define DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa (101325.0f) // True standard sea-level pressure in Pascals

static const char *TAG = "Remote_Sensor_Slave";

// ⚠️ REPLACE THIS ARRAY WITH THE EXACT PHYSICAL MAC ADDRESS OF YOUR MASTER BOARD 1
static uint8_t master_mac_address[6] = {0x70, 0xAF, 0x09, 0x3B, 0xD8, 0x8C}; 

static dps310_t dps_sensor_dev;
static mpu6050_dev_t mpu6050_sensor_dev; // Global IMU device profile handle

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


// 🎯 UPDATE THIS FUNCTION AT LINE 30 IN YOUR SLAVE FILE:
void on_data_sent_callback(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI("LINK_STATUS", "🚀 Packet delivered and ACKed by Master successfully!");
    } else {
        ESP_LOGW("LINK_STATUS", "❌ Delivery Failed! Master didn't ACK (Out of range or turned off)");
    }
}


void init_wifi_esp_now(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default()); 
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    
    // 🎯 BIND THE NEW ACK CALLBACK HERE:
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent_callback));
    
    esp_now_peer_info_t peer_info = {0}; 
    memcpy(peer_info.peer_addr, master_mac_address, 6);
    peer_info.channel = 1;               
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;       
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
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
    dps_config.tmp_oversampling = DPS310_PM_PRC_128; // Max hardware noise filtering
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

    // 🛠️ Wait for the sensor and internal coefficients matrices to become ready on the bus
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

    // 🛠️ Extract and apply factory-fused internal calculation coefficients
    ESP_ERROR_CHECK(dps310_get_coef(&dps_sensor_dev));

    // 🛠️ Calibrate absolute altitude baseline lookup metrics to 18.288 meters (60ft)
    ESP_ERROR_CHECK(dps310_calibrate_altitude(&dps_sensor_dev, 18.288f));

    // 🛠️ Spin up the background tracking pipeline engines using the proper API function
    ESP_ERROR_CHECK(dps310_backgorund_start(&dps_sensor_dev, DPS310_MODE_BACKGROUND_ALL));

    ESP_LOGI(TAG, "Slave DPS310 Barometer Initialized & Calibrated Successfully!");
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_wifi_esp_now();
    init_barometer();
    init_imu();

    float temp = 0.0, press = 0.0;
    struct esp_now_payload_t data_packet;

    mpu6050_raw_acceleration_t acce_raw = {0, 0, 0};
    mpu6050_raw_rotation_t gyro_raw = {0, 0, 0};

    while (1) {
        if (dps310_read_temp(&dps_sensor_dev, &temp) == ESP_OK && 
            dps310_read_pressure(&dps_sensor_dev, &press) == ESP_OK) {
            
            data_packet.temp = temp;
            data_packet.press_hpa = press / 100.0f;

            // // Broadcast data packet directly to the Master's MAC address
            // esp_now_send(master_mac_address, (uint8_t *)&data_packet, sizeof(data_packet));
            // ESP_LOGI(TAG, "Sent -> Temp: %.2f C | Press: %.2f hPa", data_packet.temp, data_packet.press_hpa);
        }

        // Call the strict library functions
        if (mpu6050_get_raw_acceleration(&mpu6050_sensor_dev, &acce_raw) == ESP_OK &&
            mpu6050_get_raw_rotation(&mpu6050_sensor_dev, &gyro_raw) == ESP_OK) {
            
            // Map the parsed float parameters to your over-the-air payload structure
            data_packet.acc_x = (float)acce_raw.x;
            data_packet.acc_y = (float)acce_raw.y;
            data_packet.acc_z = (float)acce_raw.z;
            
            data_packet.gyro_x = (float)gyro_raw.x;
            data_packet.gyro_y = (float)gyro_raw.y;
            data_packet.gyro_z = (float)gyro_raw.z;

        } else {
            ESP_LOGW("IMU_LOOP", "MPU6050 transaction dropped on I2C line.");
        }

        // 🚀 Dispatch the newly expanded 32-byte payload via Unicast
        esp_err_t send_result = esp_now_send(master_mac_address, (uint8_t *)&data_packet, sizeof(data_packet));

        if (send_result == ESP_OK) {
            // Slave Telemetry Matrix Printout scaled to standard units [0x1.5]
            /* ⚠️ Important Data Notice: 
                We do not apply the division to the actual over-the-air payload 
                parameters (data_packet.acc_x = acce_raw.x;) (0x1.5). We intentionally 
                keep the data transmitted across the radio as compact 16-bit integers 
                to preserve transmission speeds and prevent floating-point calculation 
                overhead across the ESP-NOW air channels (0x1.1). 
            */
            ESP_LOGI(TAG, "Sent -> Temp: %.2f C | Press: %.2f hPa | Accel: [%.2fg, %.2fg, %.2fg] | Gyro: [%.1f°/s, %.1f°/s, %.1f°/s]", 
                 data_packet.temp, 
                 data_packet.press_hpa,
                 data_packet.acc_x / 16384.0f, 
                 data_packet.acc_y / 16384.0f, 
                 data_packet.acc_z / 16384.0f,
                 data_packet.gyro_x / 131.0f, 
                 data_packet.gyro_y / 131.0f, 
                 data_packet.gyro_z / 131.0f);

        } else {
            ESP_LOGE(TAG, "Unicast Data Dispatch Blocked! Reason: %s", esp_err_to_name(send_result));
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
