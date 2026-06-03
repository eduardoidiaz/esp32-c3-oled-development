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

#include "deca_device_api.h"
#include "deca_interface.h"
#include "dw3000_deca_regs.h"

#include "dw3000_hw.h"      // Maps dw3000_hw_init() and dw3000_hw_reset()
#include "dw3000_spi.h"     // Maps your underlying SPI peripheral bindings

void print_dw3000_config(const char* node_name) {
    ESP_LOGI("UWB_DIAG", "=================================================");
    ESP_LOGI("UWB_DIAG", "📡 CONFIGURATION MATRIX FOR NODE: %s", node_name);

    // 1. Read Device ID
    uint32_t dev_id = dwt_read_reg(DEV_ID_ID); // [0x1.1]
    ESP_LOGI("UWB_DIAG", "• Device ID: 0x%08X", (unsigned int)dev_id);

    // 2. Read Radio Channel & Preamble configurations from CHAN_CTRL (0x10014)
    uint32_t chan_ctrl = dwt_read_reg(CHAN_CTRL_ID); // [0x1.13]
    uint8_t rf_chan = (chan_ctrl & CHAN_CTRL_RF_CHAN_BIT_MASK) ? 9 : 5; // Bit 0 determines Chan 5 vs 9 [0x1.13]
    uint8_t tx_code = (chan_ctrl & CHAN_CTRL_TX_PCODE_BIT_MASK) >> CHAN_CTRL_TX_PCODE_BIT_OFFSET; // Bits 3-7 [0x1.13]
    uint8_t rx_code = (chan_ctrl & CHAN_CTRL_RX_PCODE_BIT_MASK) >> CHAN_CTRL_RX_PCODE_BIT_OFFSET; // Bits 8-12 [0x1.13]

    ESP_LOGI("UWB_DIAG", "• Radio Channel: Channel %d", rf_chan);
    ESP_LOGI("UWB_DIAG", "• Preamble Codes: TX Code = %d | RX Code = %d", tx_code, rx_code);

    // 3. Read System Configurations from SYS_CFG (0x10)
    uint32_t sys_cfg = dwt_read_reg(SYS_CFG_ID); // [0x1.2]
    // Check if the 6.8 Mbps PHR bit is set [0x1.3]
    const char* rate_str = (sys_cfg & SYS_CFG_PHR_6M8_BIT_MASK) ? "6.8 Mbps" : "850 Kbps"; 
    // Check if Frame Filtering engine flag is globally enabled in hardware [0x1.3]
    const char* ff_str = (sys_cfg & SYS_CFG_FFEN_BIT_MASK) ? "ENABLED" : "DISABLED";

    ESP_LOGI("UWB_DIAG", "• PHY Header Data Rate Mode: %s", rate_str);
    ESP_LOGI("UWB_DIAG", "• Global Frame Filter Engine: %s", ff_str);

    // 4. Read Detailed Address Filter Rules from ADR_FILT_CFG (0x14)
    uint32_t adr_filt = dwt_read_reg(ADR_FILT_CFG_ID); // [0x1.3]
    ESP_LOGI("UWB_DIAG", "• Raw Address Filter Rules Matrix (ADR_FILT_CFG): 0x%04X", (unsigned int)(adr_filt & 0xFFFF));
    ESP_LOGI("UWB_DIAG", "=================================================");
}

// 🛠️ FIXING THE DRIVER HEADER BUG ON THE SLAVE EXPLICITLY:
#undef DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa
#define DPS310_AVERAGE_SEA_LEVEL_PRESSURE_Pa (101325.0f) // True standard sea-level pressure in Pascals

// Define your Slave's physical SPI pin mapping
#define PIN_UWB_MISO   2
#define PIN_UWB_MOSI   7
#define PIN_UWB_SCK    1 
#define PIN_UWB_CS     10 
#define PIN_UWB_RST    3
#define PIN_UWB_IRQ    4

static const char *TAG = "Remote_Sensor_Slave";

// ⚠️ REPLACE THIS ARRAY WITH THE EXACT PHYSICAL MAC ADDRESS OF YOUR MASTER BOARD 1
static uint8_t master_mac_address[6] = {0x70, 0xAF, 0x09, 0x3B, 0xD8, 0x8C}; 

static dps310_t dps_sensor_dev;
static mpu6050_dev_t mpu6050_sensor_dev; // Global IMU device profile handle
// Bind these to your custom esp_spi read/write functions like you did on the anchor!
extern struct dwt_spi_s dw3000_spi; 

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

// Use the exact same channel configurations (Channel 5, 6.8 Mbps) as your Anchor!
static dwt_config_t rx_config = {
    .chan           = 5,                // Channel number (6.5 GHz)
    .txPreambLength = DWT_PLEN_128,     // Preamble length
    .rxPAC          = DWT_PAC8,         // PAC size
    .txCode         = 9,                // Preamble Code 9 (16 MHz PRF)
    .rxCode         = 9,                // Preamble Code 9 (16 MHz PRF)
    .sfdType        = 1,                // 1 for DW 8-bit short SFD
    .dataRate       = DWT_BR_850K,      // 850 Kbps long-range mode
    .phrMode        = DWT_PHRMODE_STD,  // Standard PHY header
    .phrRate        = DWT_PHRRATE_STD,  // Standard PHY rate
    
    // 🎯 FIX: Expand the SFD Timeout window to handle slow 850 Kbps symbol durations!
    // Adding 1 to the preamble length configuration prevents premature hardware dropouts.
    .sfdTO          = (128 + 1 + 8 - 8), 
    
    .stsMode        = DWT_STS_MODE_OFF, // STS off
    .stsLength      = DWT_STS_LEN_64,   // Standard STS buffer layout
    .pdoaMode       = DWT_PDOA_M0       // Using your header's explicit identifier
};




#define RX_BUF_LEN 128
static uint8_t rx_buffer[RX_BUF_LEN];

extern const struct dwt_probe_s dw3000_probe_interf;

void uwb_slave_rx_task(void *pvParameters) {
    ESP_LOGI("UWB_RX", "Initializing native component hardware layers..."); //

    // 1. Initialize physical pins and single-instance SPI bus
    int hw_err = dw3000_hw_init(); //
    if (hw_err != ESP_OK) {
        ESP_LOGE("UWB_RX", "🛑 CRITICAL: Hardware layer boundary setup failed! Code: %d", hw_err); //
        vTaskDelete(NULL); //
        return; //
    }

    // 2. Clear out transceiver logic states
    dw3000_hw_reset(); //
    vTaskDelay(pdMS_TO_TICKS(50)); //

    // 3. Run the driver matching probe routine
    ESP_LOGI("UWB_RX", "Probing interface layout driver mappings..."); //
    int32_t probe_rc = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf); //
    if (probe_rc != DWT_SUCCESS) {
        ESP_LOGE("UWB_RX", "🛑 CRITICAL: Qorvo DW3000 interface probe table layout failed! RC: %ld", probe_rc); //
        vTaskDelete(NULL); //
        return; //
    }

    // 4. Run the device register configuration script
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { //
        ESP_LOGE("UWB_RX", "🛑 CRITICAL: DW3000 internal register configuration failed!"); //
        vTaskDelete(NULL); //
        return; //
    }

    // 5. Apply radio channel settings and turn on the receiver engine
    dwt_configure(&rx_config);

    print_dw3000_config("SLAVE_NODE");
    
    // 🎯 Set target address filters for validation 
    dwt_setpanid(0xDECA);
    dwt_setaddress16(0x0002);

    // 🎯 Configure frame filter to accept everything (Bypass strict verification)
    dwt_configureframefilter(0x0, 0x3FF);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    ESP_LOGI("UWB_RX", "🎯 SUCCESS: Slave Transceiver operational and listening...");

   uint32_t loop_counter = 0; // [0x1.4]

    while (1) { // [0x1.4]
        // Read the live system status register
        uint32_t status_reg = dwt_read_reg(SYS_STATUS_ID); // [0x1.4]
        
        if (loop_counter++ % 150 == 0) { // [0x1.4]
            ESP_LOGW("UWB_STATUS_DEBUG", "Live SYS_STATUS Register: 0x%08X", (unsigned int)status_reg); // [0x1.4]
        }

        // 1. Check if a frame is fully ready inside the hardware FIFO
        // Using RXFR captures packets even if the CRC trailing bytes fluctuate
        if (status_reg & (DWT_INT_RXFCG_BIT_MASK | 0x00010000)) { 
            
            uint32_t rx_finfo = dwt_read_reg(RX_FINFO_ID); // [0x1.4]
            uint32_t frame_len = rx_finfo & RX_FINFO_RXFLEN_BIT_MASK; // [0x1.4]
            
            if (frame_len <= RX_BUF_LEN && frame_len > 11) { // [0x1.5]
                // Clear your tracking buffer completely before copying new bytes
                memset(rx_buffer, 0, sizeof(rx_buffer));
                
                // Read the exact amount of bytes delivered over the air
                dwt_readrxdata(rx_buffer, frame_len, 0); // [0x1.5]
                
                // Calculate actual payload length (subtract 9 bytes of headers and 2 bytes of CRC)
                int payload_len = (int)frame_len - 9 - 2;
                
                // Ensure we only print if the payload length matches exactly what we expect ("PING" = 4 bytes)
                if (payload_len == 4) {
                    char text_payload[5] = {0};
                    memcpy(text_payload, rx_buffer + 9, 4);
                    text_payload[4] = '\0';
                    
                    // Verify the content matches our transmission string sequence
                    if (text_payload[0] == 'P' && text_payload[1] == 'I') {
                        ESP_LOGI("UWB_RX", "🎉 Packet Received! Size: %ld | Data: %s", frame_len, text_payload);
                    }
                }
            }
            
            // 2. Clear out ALL active status bits dynamically to reset internal state machines
            dwt_write_reg(SYS_STATUS_ID, status_reg | 0xFFFFFFFF); 
            
            // 3. Free up the dual-bank hardware buffers safely
            dwt_signal_rx_buff_free(); // [0x1.38]
            
            // 4. Force a settling window and re-arm the receiver frontend
            esp_rom_delay_us(10);
            dwt_rxenable(DWT_START_RX_IMMEDIATE); // [0x1.5]
        }
        
        // Handle common frame reception timeout errors or broken payload CRC faults cleanly
        else if (status_reg & (DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | 0x04000000)) { // [0x1.5]
            dwt_write_reg(SYS_STATUS_ID, status_reg | 0xFFFFFFFF); 
            dwt_signal_rx_buff_free();
            esp_rom_delay_us(10);
            dwt_rxenable(DWT_START_RX_IMMEDIATE); // [0x1.5]
        }
        
        vTaskDelay(1); // [0x1.5]
    }
}




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

    xTaskCreate(
        uwb_slave_rx_task,    // Function pointer to your UWB receiver task loop
        "uwb_slave_rx_task",  // Diagnostic text name for FreeRTOS
        4096,                 // Stack memory size (4KB is safe for UWB operations)
        NULL,                 // Task input parameters
        5,                    // Priority lane (Keeps it high enough to capture radio frames)
        NULL                  // Task handle pointer
    );
    ESP_LOGI("SLAVE_MAIN", "🚀 UWB Slave RX Task successfully registered into active lane!");

    struct esp_now_payload_t data_packet = {0};
    
    // 🎯 1. ALLOCATE THE EMA FILTER MEMORY TRACKERS
    // Static variables preserve their values between loop iterations
    static float filtered_acc_x = 0.0f;
    static float filtered_acc_y = 0.0f;
    static float filtered_acc_z = 0.0f;
    
    // Set your smoothing coefficient (15% new data, 85% historical smoothness)
    const float imu_alpha = 0.25f; 
    
    // Flag to handle the very first packet cleanly without a lag spike
    static bool first_run = true;

    while (1) {
        // Fetch Barometer Metrics as usual...
        float temp = 0.0f, press = 0.0f;
        if (dps310_read_pressure(&dps_sensor_dev, &press) == ESP_OK &&
            dps310_read_temp(&dps_sensor_dev, &temp) == ESP_OK) {
            data_packet.temp = temp;
            data_packet.press_hpa = press / 100.0f;
        }

        // Fetch Raw MPU6050 Metrics
        mpu6050_raw_acceleration_t acce_raw = {0, 0, 0};
        mpu6050_raw_rotation_t gyro_raw = {0, 0, 0};

        if (mpu6050_get_raw_acceleration(&mpu6050_sensor_dev, &acce_raw) == ESP_OK &&
            mpu6050_get_raw_rotation(&mpu6050_sensor_dev, &gyro_raw) == ESP_OK) {
            
            float raw_x = (float)acce_raw.x;
            float raw_y = (float)acce_raw.y;
            float raw_z = (float)acce_raw.z;

            // 🎯 2. APPLY THE EMA FILTER PATTERN
            if (first_run) {
                // Seed the filter with initial hardware states on boot
                filtered_acc_x = raw_x;
                filtered_acc_y = raw_y;
                filtered_acc_z = raw_z;
                first_run = false;
            } else {
                // Execute the standard moving average math
                filtered_acc_x = (imu_alpha * raw_x) + ((1.0f - imu_alpha) * filtered_acc_x);
                filtered_acc_y = (imu_alpha * raw_y) + ((1.0f - imu_alpha) * filtered_acc_y);
                filtered_acc_z = (imu_alpha * raw_z) + ((1.0f - imu_alpha) * filtered_acc_z);
            }

            // 🎯 3. PACK THE SMOOTHED MEMORY PATHS INTO THE RADIO PAYLOAD
            data_packet.acc_x = filtered_acc_x;
            data_packet.acc_y = filtered_acc_y;
            data_packet.acc_z = filtered_acc_z;
            
            // Gyroscope values typically pass unfiltered, or can use their own EMA handles
            data_packet.gyro_x = (float)gyro_raw.x;
            data_packet.gyro_y = (float)gyro_raw.y;
            data_packet.gyro_z = (float)gyro_raw.z;
            
        }

        // Send the data packet over the air
        esp_now_send(master_mac_address, (uint8_t *)&data_packet, sizeof(data_packet));
        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
