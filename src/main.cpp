/**
 * @file main.c
 * @brief Main application for ESP32-C6 Weather & Soil Sensor Node using ESP-IDF.
 * * This file contains the main logic for reading sensor data (DHT11, Thermistor, Soil Moisture, GPS)
 * and transmitting it via a LoRa module.
 * * It is a conversion of an Arduino project to the native ESP-IDF framework.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"

// Custom component includes (assuming they are in the 'components' directory)
#include "dht.h"
#include "lora.h"
#include "tiny_gps_plus.h" // A C-compatible wrapper or C library for GPS parsing

// --- Pin Definitions ---
// DHT11 Sensor
#define DHT_PIN GPIO_NUM_22

// GPS Module (UART1)
#define GPS_TX_PIN GPIO_NUM_4
#define GPS_RX_PIN GPIO_NUM_5
#define GPS_UART_PORT UART_NUM_1
#define GPS_BAUD_RATE 9600

// Thermistor (ADC1)
#define THERMISTOR_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO0 for ADC1 Channel 0

// Soil Moisture Sensor (ADC1)
#define SOIL_ADC_CHANNEL ADC1_CHANNEL_3 // GPIO3 for ADC1 Channel 3

// LoRa Module (SPI)
#define LORA_NSS_PIN GPIO_NUM_18
#define LORA_RST_PIN GPIO_NUM_19
#define LORA_DI0_PIN GPIO_NUM_20
#define LORA_SCK_PIN GPIO_NUM_6
#define LORA_MISO_PIN GPIO_NUM_2
#define LORA_MOSI_PIN GPIO_NUM_7

// --- Constants ---
static const char *TAG = "WEATHER_STATION";

// Thermistor Constants
const float SERIES_RESISTOR = 10000.0f;
const float NOMINAL_RESISTANCE = 10000.0f;
const float NOMINAL_TEMPERATURE = 25.0f;
const float BETA_COEFFICIENT = 3892.0f;
const float VREF = 3300; // ADC reference voltage in mV

// Soil Data Constants
float soil_alpha = 0.15f;
float soil_smoothed = 0.0f;
int32_t soil_dry_adc = -1;
int32_t soil_wet_adc = -1;
#define NVS_STORAGE_NAMESPACE "storage"

// GPS Constants & Objects
#define GPS_BUFFER_SIZE 1024
TinyGPSPlus gps;

// --- Global Variables for Sensor Data ---
float inside_temp = 0.0f;
float inside_hum = 0.0f;
float gps_lat = 0.0f;
float gps_lon = 0.0f;
float soil_moisture = 0.0f;
float outside_temp = 0.0f;

// ADC handle and characteristics for calibration
static esp_adc_cal_characteristics_t adc_chars;

// --- Function Prototypes ---
void save_calibration();
void load_calibration();
void gps_task(void *pvParameters);
void lora_task(void *pvParameters);

// --- NVS (EEPROM Replacement) Functions ---
void save_calibration() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_i32(nvs_handle, "soil_dry", soil_dry_adc);
    nvs_set_i32(nvs_handle, "soil_wet", soil_wet_adc);
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed!");
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Calibration values saved to NVS.");
}

void load_calibration() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for reading: %s", esp_err_to_name(err));
        return;
    }
    nvs_get_i32(nvs_handle, "soil_dry", &soil_dry_adc);
    nvs_get_i32(nvs_handle, "soil_wet", &soil_wet_adc);
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded calibration: Dry=%d, Wet=%d", (int)soil_dry_adc, (int)soil_wet_adc);
}

// --- Helper Functions ---
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- Sensor Reading Functions ---
float read_outside_temp() {
    uint32_t adc_reading = 0;
    // Read ADC and convert to voltage
    adc_reading = adc1_get_raw(THERMISTOR_ADC_CHANNEL);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

    if (voltage <= 0) return -273.15f;
    
    float vout = (float)voltage / 1000.0f;
    float vref_f = VREF / 1000.0f;
    
    float denom = (vref_f - vout);
    if (denom <= 0.0f) return 150.0f;
    
    float r_therm = SERIES_RESISTOR * (vout / denom);
    
    float t0 = NOMINAL_TEMPERATURE + 273.15f;
    float invT = (1.0f / t0) + (1.0f / BETA_COEFFICIENT) * log(r_therm / NOMINAL_RESISTANCE);
    if (invT <= 0.0f) return -273.15f;
    
    float t_kelvin = 1.0f / invT;
    float tempC = t_kelvin - 273.15f;
    
    return tempC;
}

void read_inside_dht() {
    float humidity, temperature;
    if (dht_read_float_data(DHT_TYPE_DHT11, DHT_PIN, &humidity, &temperature) == ESP_OK) {
        inside_temp = temperature;
        inside_hum = humidity;
        ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", inside_temp, inside_hum);
    } else {
        ESP_LOGE(TAG, "Could not read data from DHT11 sensor");
    }
}

float read_soil_moisture() {
    int raw = adc1_get_raw(SOIL_ADC_CHANNEL);
    
    if (soil_smoothed == 0.0f) soil_smoothed = raw;
    soil_smoothed = soil_alpha * raw + (1.0f - soil_alpha) * soil_smoothed;
    
    if (soil_dry_adc >= 0 && soil_wet_adc >= 0 && soil_dry_adc != soil_wet_adc) {
        float pct = map_float(soil_smoothed, soil_dry_adc, soil_wet_adc, 0.0f, 100.0f);
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        return pct;
    } else {
        return -1.0f; // Not calibrated
    }
}


// --- Tasks ---
void gps_task(void *pvParameters) {
    uint8_t* data = (uint8_t*) malloc(GPS_BUFFER_SIZE);
    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, data, GPS_BUFFER_SIZE, 20 / portTICK_RATE_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                gps.encode(data[i]);
            }
        }
        
        if (gps.location.isUpdated()) {
            gps_lat = gps.location.lat();
            gps_lon = gps.location.lng();
            ESP_LOGI(TAG, "GPS Location: Lat=%.6f, Lon=%.6f", gps_lat, gps_lon);
        }
        
        vTaskDelay(10 / portTICK_RATE_MS); // Yield to other tasks
    }
}

void main_task(void *pvParameters) {
    int lora_counter = 0;

    while (1) {
        // --- Read all sensors ---
        read_inside_dht();
        outside_temp = read_outside_temp();
        soil_moisture = read_soil_moisture();

        ESP_LOGI(TAG, "Out Temp: %.2f°C", outside_temp);
        if (soil_moisture >= 0.0f) {
            ESP_LOGI(TAG, "Moisture: %.1f%%", soil_moisture);
        } else {
            ESP_LOGW(TAG, "Moisture: UNCALIBRATED");
        }
        
        // --- Prepare and Send LoRa Packet ---
        char payload[150];
        snprintf(payload, sizeof(payload), 
            "InTemp=%.1fC,InHum=%.1f%%,Lat=%.6f,Lon=%.6f,Soil=%.1f%%,OutTemp=%.1fC,Count=%d",
            inside_temp, inside_hum, gps_lat, gps_lon, soil_moisture, outside_temp, lora_counter++);

        lora_send_packet((uint8_t *)payload, strlen(payload));
        ESP_LOGI(TAG, "LoRa packet sent: %s", payload);

        // --- Handle Incoming LoRa Data (optional) ---
        // You can add logic here to check for received packets if needed.
        
        // --- Delay for the next cycle ---
        vTaskDelay(10000 / portTICK_RATE_MS); // Wait 10 seconds
    }
}


// --- Initialization Functions ---
void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS Initialized.");
}

void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(THERMISTOR_ADC_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(SOIL_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    // Characterize ADC
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC using eFuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC using Two Point eFuse");
    } else {
        ESP_LOGI(TAG, "ADC using default Vref");
    }
}

void init_uart_gps() {
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(GPS_UART_PORT, GPS_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_lora() {
    lora_init();
    lora_set_pins(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN, LORA_RST_PIN, LORA_DI0_PIN);
    
    if (!lora_begin(433E6)) {
        ESP_LOGE(TAG, "Starting LoRa failed!");
        // You might want to halt or retry here
        while(1);
    }
    lora_set_spreading_factor(8);
    ESP_LOGI(TAG, "LoRa Initialized.");
}


// --- Main Application Entry Point ---
void app_main(void) {
    ESP_LOGI(TAG, "Initializing application...");

    // Initialize NVS for storing calibration data
    init_nvs();
    
    // Load previously saved calibration data
    load_calibration();
    
    // Initialize peripherals
    init_adc();
    init_uart_gps();
    init_lora();

    // Start background tasks
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Initialization complete. Main tasks running.");
    
    // Note: The calibration from your original code was triggered by serial input.
    // In a real-world scenario, you might use a button press or a specific LoRa command
    // to trigger set_soil_dry() and set_soil_wet() and then call save_calibration().
    // For now, these would need to be integrated into the logic as needed.
}
