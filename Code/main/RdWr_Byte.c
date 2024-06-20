#include "driver/i2c.h"
#include "vl53l1_platform.h"

#define I2C_MASTER_SCL_IO           19          // SCL pin
#define I2C_MASTER_SDA_IO           18          // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ          400000      // I2C frequency
#define VL53L1X_I2C_ADDRESS         0x29        // Default I2C address of VL53L1X (7-bit address)

#define I2C_MASTER_NUM I2C_NUM_0 // Use I2C port 0

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (index >> 8) & 0xFF, true); // MSB of the register address
    i2c_master_write_byte(cmd, index & 0xFF, true); // LSB of the register address
    i2c_master_start(cmd); // Repeated start condition
    i2c_master_write_byte(cmd, (VL53L1X_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (index >> 8) & 0xFF, true); // MSB of the register address
    i2c_master_write_byte(cmd, index & 0xFF, true); // LSB of the register address
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t msb, lsb;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = ESP_FAIL;

    // Start condition and device address + write bit
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    // Register address (high byte and then low byte)
    i2c_master_write_byte(cmd, (index >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, index & 0xFF, true);

    // Repeated start condition, device address + read bit
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L1X_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

    // Read MSB and LSB
    i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute the command
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *data = ((uint16_t)msb << 8) | (uint16_t)lsb;
        return 0;
    } else {
        return -1;
    }
}
