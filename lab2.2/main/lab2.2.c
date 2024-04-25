#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define CRC8_POLY 0x31 // CRC8 polynomial for SHTC3 sensor
#define I2C_MASTER_SCL_IO 8 
#define I2C_MASTER_SDA_IO 10 
#define I2C_MASTER_NUM I2C_NUM_0 
#define I2C_MASTER_TX_BUF_DISABLE 0 
#define I2C_MASTER_RX_BUF_DISABLE 0 
#define SHTC3_SENSOR_ADDR 0x70 
#define WRITE_BIT I2C_MASTER_WRITE 
#define READ_BIT I2C_MASTER_READ 
#define ACK_CHECK_EN 0x1 
#define ACK_CHECK_DIS 0x0 
#define ACK_VAL 0x0 
#define NACK_VAL 0x1 
#define I2C_MASTER_FREQ_HZ 400000

static const char *TAG = "SHTC3";
static esp_err_t power_up_sensor() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x35, true); // Power up command
    i2c_master_write_byte(cmd, 0x17, true); // Power up command
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Function to power down the sensor
static esp_err_t power_down_sensor() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xB0, true); // Power down command
    i2c_master_write_byte(cmd, 0x98, true); // Power down command
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int byte = 0; byte < len; byte++) {
        crc ^= data[byte];
        for (int bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

static esp_err_t i2c_master_init(){
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                               I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t read_humidity(float *humidity) {
    uint8_t data[6];
    esp_err_t ret;
    // Command to start temperature measurement
    uint8_t cmd_start_temp[] = {0x35, 0x17};
    // Start I2C communication
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_start_temp, sizeof(cmd_start_temp), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for measurement
    // Command to read temperature data
    uint8_t cmd_read_temp[] = {0x5C, 0x24};
    // Start I2C communication for reading temperature data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_read_temp, sizeof(cmd_read_temp), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Calculate CRC8 checksum
    uint8_t received_crc = data[2];
    uint8_t calculated_crc = crc8(data, 2);
    if (received_crc != calculated_crc) {
        return ESP_ERR_INVALID_CRC;
    }
    // If CRC8 matches, proceed with temperature calculation
    uint16_t humidity_raw = (data[0] << 8) | data[1];
    *humidity= 100 * ((float)humidity_raw / 65535.0f);
    return ESP_OK;
}

static esp_err_t read_temperature(float *temperature) {
    uint8_t data[6];
    esp_err_t ret;
    // Command to start temperature measurement
    uint8_t cmd_start_temp[] = {0x35, 0x17};
    // Start I2C communication
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_start_temp, sizeof(cmd_start_temp), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for measurement
    // Command to read temperature data
    uint8_t cmd_read_temp[] = {0x7C, 0xA2};
    // Start I2C communication for reading temperature data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, cmd_read_temp, sizeof(cmd_read_temp), true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    // Calculate CRC8 checksum
    uint8_t received_crc = data[2];
    uint8_t calculated_crc = crc8(data, 2);
    if (received_crc != calculated_crc) {
        return ESP_ERR_INVALID_CRC;
    }
    // If CRC8 matches, proceed with temperature calculation
    uint16_t temp_raw = (data[0] << 8) | data[1];
    *temperature = -45 + 175 * ((float)temp_raw / 65535.0f);
    return ESP_OK;
}

void app_main() {
    // Initialize I2C master
    esp_err_t i2c_init_ret = i2c_master_init();
    if (i2c_init_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master");
        return;
    }
    // Power up the sensor initially
    esp_err_t power_up_ret = power_up_sensor();
    if (power_up_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up the sensor");
        return;
    }
    // Main loop
    while (1) {
        float temperature = 0.0;
        // Read temperature
        esp_err_t temp_read_ret = read_temperature(&temperature);
        if (temp_read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read temperature");
            continue;
        }
        // Print temperature
        ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);
        float humidity = 0.0;
        // Read humidity
        esp_err_t humidity_read_ret = read_humidity(&humidity);
        if (humidity_read_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read humidity");
            continue;
        }
        // Print humidity
        ESP_LOGI(TAG, "Humidity: %.2f%%", humidity);
        // Power down the sensor before delay
        esp_err_t power_down_ret = power_down_sensor();
        if (power_down_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to power down the sensor");
            continue;
        }
        // Wait before next reading
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        // Power up the sensor after delay
        power_up_ret = power_up_sensor();
        if (power_up_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to power up the sensor");
            return;
        }
    }
}
