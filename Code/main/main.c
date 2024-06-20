/* Code for the RoboticWorx Gateway smartwatch project! Build this project for yourself at www.roboticworx.io. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <freertos/timers.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "st7789.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_png.h"
#include "pngle.h"

#include <string.h>
#include "math.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "VL53L1X_api.h"

#include <bme680.h>
#include <icm42670.h>
#include <ds3231.h>
#include <mcp342x.h>

SemaphoreHandle_t i2cMutex;

#define INTERVAL 400
#define WAIT vTaskDelay(INTERVAL)

#define COBALT rgb565(0, 71, 171) // Cobalt blue color
#define DESERT_ORANGE rgb565(253, 164, 90) // Desert orange color
#define DESERT_RED rgb565(204, 85, 57) // Desert red color
#define DESERT_BLUE rgb565(119, 181, 254) // Desert blue color
#define DESERT_GREEN rgb565(189, 204, 150) // Desert blue color
#define DESERT_WHITE rgb565(255, 255, 255) // Desert blue color
#define DESERT_BLACK rgb565(0, 0, 0) // Desert blue color

#define I2C_MASTER_SCL_IO           14          // SCL pin
#define I2C_MASTER_SDA_IO           13          // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number
#define I2C_MASTER_FREQ_HZ          100000      // I2C frequency (100KHz)
#define VL53L1X_I2C_ADDRESS         0x29        // Default I2C address of VL53L1X (7-bit address)
#define ICM42670_I2C_ADDRESS        0x69
#define BME680_I2C_ADDRESS          0x76
#define MCP3427_I2C_ADDRESS         0x6E

#define I2C_MASTER_NUM I2C_NUM_0 // Use I2C port 0

#define GAIN  MCP342X_GAIN1       // +-2.048
#define CHANNEL MCP342X_CHANNEL1
#define RESOLUTION MCP342X_RES_16 // 16-bit, 15 sps

#define BUTTON_1 18
#define BUTTON_2 48
#define BUTTON_3 47
#define BUTTON_4 16
#define BUTTON_5 17
#define BACKLIGHT_PIN 7
#define CHARGE_PIN 21
#define LASER_PIN 6
#define WOM_PIN 15
#define LIDAR_PIN 38

#define MAX_PEERS 20  // Maximum number of peers to track
#define DEFAULT_SCAN_LIST_SIZE 5

#define SLEEP_TIMEOUT_MS 1500

#define MAX_MAC_ADDRESSES 11
#define MAC_ADDRESS_LENGTH 12

//#define RECEIVER_MAC_ADDR {0x68, 0xB6, 0xB3, 0x2C, 0x12, 0x00} // 68:b6:b3:2c:12:00
//static const uint8_t receiver_mac[] = RECEIVER_MAC_ADDR; // Use static const for fixed values

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_0)
#define ADDR BME680_I2C_ADDR_0
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_1)
#define ADDR BME680_I2C_ADDR_1
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG_ST7789 = "ST7789";
static const char *TAG_VL53L1X = "VL53L1X";
static const char *TAG_ICM42670 = "ICM42670";
static const char *TAG_TIMER = "Sleep Timer";

static QueueHandle_t bmeQueue = NULL;
static QueueHandle_t lidarQueue = NULL;
static QueueHandle_t timeQueue = NULL;
static QueueHandle_t ICMQueue = NULL;
static QueueHandle_t MCPQueue = NULL;
static QueueHandle_t setTimeQueue = NULL;
static QueueHandle_t wirelessQueue = NULL;

TaskHandle_t getBME680Handle = NULL;
TaskHandle_t getVL53L1XHandle = NULL;
TaskHandle_t getDS3231Handle = NULL;
TaskHandle_t getICM42670Handle = NULL;
TaskHandle_t getMCP3427Handle = NULL;
TaskHandle_t sendWirelessHandle = NULL;

typedef struct {
    float temperature;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm42670_data_t;

icm42670_data_t imuData;

static mcp342x_t adc;
static TimerHandle_t sleep_timer;
static void SPIFFS_Directory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

// You have to set these CONFIG value using menuconfig.
#if 0
#define CONFIG_WIDTH  240
#define CONFIG_HEIGHT 280
#define CONFIG_MOSI_GPIO 11
#define CONFIG_SCLK_GPIO 12
#define CONFIG_CS_GPIO 13
#define CONFIG_DC_GPIO 14
#define CONFIG_RESET_GPIO 15
#define CONFIG_BL_GPIO 16
#endif

volatile int currentScreen = 0;
volatile int selectedTime = 1;

volatile int selectedDigit = 1;
volatile int selectedMAC = 1;

bool sleepTimerDone = false;

volatile bool backlightOn = false;
volatile bool sleepTimerOn = true;
volatile bool laserOn = false;
volatile bool lidarOn = false;

uint8_t macaddresses[MAX_MAC_ADDRESSES][MAC_ADDRESS_LENGTH];

TickType_t last_wakeup = 0;
bool updateTime = false;

void init_wireless(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize the event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set Wi-Fi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

}

void deinit_wireless(void) {

    // Stop Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_stop());

    // Deinitialize Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_deinit());

    // Deinitialize the event loop
    ESP_ERROR_CHECK(esp_event_loop_delete_default());

    // Deinitialize NVS
    ESP_ERROR_CHECK(nvs_flash_deinit());

}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //ESP_LOGI(TAG, "Data sent with status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    //if (status == ESP_NOW_SEND_SUCCESS) {
        // Signal that data was sent successfully
    //}
    //printf("stat: %d\n", dataSentSuccessfully);
}


void add_peer(const uint8_t *mac_address) {
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false; // Change to true if encryption is needed
    memcpy(peerInfo.peer_addr, mac_address, ESP_NOW_ETH_ALEN);

    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

void reset_sleep_timer() {
	sleepTimerDone = false;
    if (xTimerReset(sleep_timer, 0) != pdPASS) {
        ESP_LOGE(TAG_TIMER, "Failed to reset sleep timer");
    }
}

static void enter_light_sleep()
{
    //esp_rom_gpio_pad_select_gpio(BACKLIGHT_PIN); // Turn off display
    gpio_set_direction(BACKLIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BACKLIGHT_PIN, 0);
    backlightOn = false;

    //gpio_set_level(LIDAR_PIN, 0);
    gpio_hold_en(BACKLIGHT_PIN); // Lock GPIO state
    gpio_hold_en(LIDAR_PIN); // Lock GPIO state

    //vTaskDelay(pdMS_TO_TICKS(300));

    // Configure the button and ICM42670 interrupt pin as wakeup sources
    uint64_t wakeup_pins = (1ULL << BUTTON_1) | (1ULL << WOM_PIN);
    esp_sleep_enable_ext1_wakeup(wakeup_pins, ESP_EXT1_WAKEUP_ANY_HIGH); // Wake up on any high level

    // Enter light sleep
    esp_light_sleep_start();

    // The device wakes up here
    //vTaskDelay(pdMS_TO_TICKS(100));

    reset_sleep_timer();

    gpio_hold_dis(BACKLIGHT_PIN); // Disable hold
    gpio_hold_dis(LIDAR_PIN);
    //gpio_set_level(BACKLIGHT_PIN, 1); // Turn display back on
    //gpio_set_level(LIDAR_PIN, 1);

}

void sleep_timer_callback(TimerHandle_t xTimer)
{
	sleepTimerDone = true;
    if (currentScreen == 0) // Only can sleep on screen 0
    {
		ESP_LOGI(TAG_TIMER, "Sleep timer expired, entering light sleep");

    	enter_light_sleep();
    }
}

static double convert_to_degrees(int value)
{
    return ((value * 90.0) / 8195.0); // + 90
}

static float convert_voltage_to_percentage(float voltage)
{
	float min_voltage = 0.94; // 0.94 actual
	float max_voltage = 1.165; // 1.7 actual

    // Convert the voltage to a percentage
    float percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;

    if (percentage > 100)
    	percentage = 100;
    else if (percentage < 1)
    	percentage = 1;

    return percentage;
}

static float convert_voltage_bat(float voltage)
{
	float r1 = 5000;
	float r2 = 2000;

    float bat_voltage = ((r1 + r2) * voltage) / r2;
    return bat_voltage;
}

TickType_t drawPNG(TFT_t * dev, char * file, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdSetFontDirection(dev, 0);

	// open PNG file
	FILE* fp = fopen(file, "rb");
	if (fp == NULL) {
		ESP_LOGW(__FUNCTION__, "File not found [%s]", file);
		return 0;
	}

	char buf[1024];
	size_t remain = 0;
	int len;

	pngle_t *pngle = pngle_new(width, height);

	pngle_set_init_callback(pngle, png_init);
	pngle_set_draw_callback(pngle, png_draw);
	pngle_set_done_callback(pngle, png_finish);

	double display_gamma = 2.2;
	pngle_set_display_gamma(pngle, display_gamma);


	while (!feof(fp)) {
		if (remain >= sizeof(buf)) {
			ESP_LOGE(__FUNCTION__, "Buffer exceeded");
			while(1) vTaskDelay(1);
		}

		len = fread(buf + remain, 1, sizeof(buf) - remain, fp);
		if (len <= 0) {
			//printf("EOF\n");
			break;
		}

		int fed = pngle_feed(pngle, buf, remain + len);
		if (fed < 0) {
			ESP_LOGE(__FUNCTION__, "ERROR; %s", pngle_error(pngle));
			while(1) vTaskDelay(1);
		}

		remain = remain + len - fed;
		if (remain > 0) memmove(buf, buf + fed, remain);
	}

	fclose(fp);

	uint16_t _width = width;
	uint16_t _cols = 0;
	if (width > pngle->imageWidth) {
		_width = pngle->imageWidth;
		_cols = (width - pngle->imageWidth) / 2;
	}
	ESP_LOGD(__FUNCTION__, "_width=%d _cols=%d", _width, _cols);

	uint16_t _height = height;
	uint16_t _rows = 0;
	if (height > pngle->imageHeight) {
			_height = pngle->imageHeight;
			_rows = (height - pngle->imageHeight) / 2;
	}
	ESP_LOGD(__FUNCTION__, "_height=%d _rows=%d", _height, _rows);
	uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _width);

#if 0
	for(int y = 0; y < _height; y++){
		for(int x = 0;x < _width; x++){
			pixel_png pixel = pngle->pixels[y][x];
			uint16_t color = rgb565(pixel.red, pixel.green, pixel.blue);
			lcdDrawPixel(dev, x+_cols, y+_rows, color);
		}
	}
#endif

	for(int y = 0; y < _height; y++){
		for(int x = 0;x < _width; x++){
			//pixel_png pixel = pngle->pixels[y][x];
			//colors[x] = rgb565(pixel.red, pixel.green, pixel.blue);
			colors[x] = pngle->pixels[y][x];
		}
		lcdDrawMultiPixels(dev, _cols, y+_rows, _width, colors);
		//vTaskDelay(1);
	}
	lcdDrawFinish(dev);
	free(colors);
	pngle_destroy(pngle, width, height);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%"PRIu32,diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

void getBME680(void *pvParameters)
{

    bme680_t sensor;
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDRESS, PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 4x oversampling for temperature
    // and 2x oversampling for humidity and pressure.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    //bme680_set_heater_profile(&sensor, 0, 200, 100);
    //bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    //bme680_set_ambient_temperature(&sensor, 25);

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    last_wakeup = xTaskGetTickCount();

    bme680_values_float_t values;
    while (1)
    {
    	uint32_t ulNotificationValue;
    	if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
    	{
			if (ulNotificationValue == 1)
			{
				// trigger the sensor to start one TPHG measurement cycle
				if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
				{
					if (bme680_force_measurement(&sensor) == ESP_OK)
					{
						// passive waiting until measurement results are available
						vTaskDelay(duration);

							// get the results and do something with them
							if (bme680_get_results_float(&sensor, &values) == ESP_OK)
							{
								//printf("sending bme data\n");
								//printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n", values.temperature, values.humidity, values.pressure, values.gas_resistance);
								xQueueSend(bmeQueue, &values, 0);
							}

					}
					xSemaphoreGive(i2cMutex);
				}
				//printf("ul is 1\n");

				// passive waiting until 1 second is over
				vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(500));
			}
    	}
    }
}

void getVL53L1X(void *pvParameters)
{

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	bool ranging = false; // If sensor is taking measurements

	gpio_set_level(LIDAR_PIN, 1);
    VL53L1_Dev_t dev; // Device struct
    uint16_t distance;
    VL53L1X_ERROR status = 0;

    // Device Initialization and configuration
    // The dev structure is passed to the API functions, but the handling of the I2C address
    // is done within your implemented communication functions
    status = VL53L1X_SensorInit(dev);
    if (status != 0) {
    	ESP_LOGE(TAG_VL53L1X, "Sensor initialization failed!");
    }

    // Set the region of interest (ROI) to a smaller area
    //VL53L1X_SetROI(dev, 4, 4);

    status = VL53L1X_StartRanging(dev); // Start ranging
    if (status != 0) {
    	ESP_LOGE(TAG_VL53L1X, "Starting ranging failed!");
    }

    uint8_t isReady = 0;
    while (1)
    {
    	uint32_t ulNotificationValue;
    	if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
    	{
			if (ulNotificationValue == 1)
			{
				if (!ranging)
				{
					VL53L1X_StartRanging(dev);
					ranging = true;
				}

				gpio_set_level(LIDAR_PIN, 1);
				vTaskDelay(pdMS_TO_TICKS(100));
				status = VL53L1X_CheckForDataReady(dev, &isReady);
				if (status == 0 && isReady && xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
				{
					status = VL53L1X_GetDistance(dev, &distance);
					if (status == 0)
					{
						ESP_LOGI(TAG_VL53L1X, "Measured distance: %d mm", distance);
						xQueueSend(lidarQueue, &distance, 0);
					}
					VL53L1X_ClearInterrupt(dev); // Prepare for the next measurement
					xSemaphoreGive(i2cMutex);
				}
			}
			else
			{
				if (ranging)
				{
					VL53L1X_StopRanging(dev);
					ranging = false;
				}
			}
    	}
    }
}

void getDS3231(void *pvParameters) {
    int year = 124; // since 1900 (2024 - 1900)
    int month = 0; // 0-based
    int day = 1;
    int hour = 0;
    int minute = 0;
    int second = 0;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO)); //sda scl

    // setup datetime: 2016-10-10 13:50:10
    struct tm time = {
        .tm_year = 124, //since 1900 (2016 - 1900)
        .tm_mon  = 11, // 0-based
        .tm_mday = 12,
        .tm_hour = 12,
        .tm_min  = 12,
        .tm_sec  = 0
    };

    ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));

    float temp;
    while (1)
    {
        uint32_t ulNotificationValue;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
        {
            if (ulNotificationValue == 1)
            {
                vTaskDelay(pdMS_TO_TICKS(150));

                if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
                {
                    if (ds3231_get_temp_float(&dev, &temp) != ESP_OK) {
                        // printf("Could not get temperature\n");
                        // continue;
                    }

                    if (ds3231_get_time(&dev, &time) != ESP_OK) {
                        // printf("Could not get time\n");
                        // continue;
                    }

                    xQueueSend(timeQueue, &time, 0);
                    xSemaphoreGive(i2cMutex);
                }

                if (currentScreen == 3)
                {
                    switch (selectedTime) {
                        case 1:
                            if (gpio_get_level(BUTTON_5) == 1) year++;
                            if (year > 199) year = 100;
                            break;
                        case 2:
                            if (gpio_get_level(BUTTON_5) == 1) month++;
                            if (month > 11) month = 0; // 0 based
                            break;
                        case 3:
                            if (gpio_get_level(BUTTON_5) == 1) day++;
                            if (day > 31) day = 1;
                            break;
                        case 4:
                            if (gpio_get_level(BUTTON_5) == 1) hour++;
                            if (hour > 23) hour = 0;
                            break;
                        case 5:
                            if (gpio_get_level(BUTTON_5) == 1) minute++;
                            if (minute > 59) minute = 0;
                            break;
                        case 6:
                            if (gpio_get_level(BUTTON_5) == 1) second++;
                            if (second > 59) second = 0;
                            break;
                    }

                    if (gpio_get_level(BUTTON_4) == 1)
                    	selectedTime++;

                    if (selectedTime > 6)
                    	selectedTime = 1;

                    struct tm setTime = {
                        .tm_year = year, // since 1900 (2016 - 1900)
                        .tm_mon  = month,  // 0-based
                        .tm_mday = day,
                        .tm_hour = hour,
                        .tm_min  = minute,
                        .tm_sec  = second,
                    };

                    if (gpio_get_level(BUTTON_4) == 1 && gpio_get_level(BUTTON_5) == 1)
                    {
                        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
                        {
                            ESP_ERROR_CHECK(ds3231_set_time(&dev, &setTime));
                            currentScreen = 0;
                            updateTime = true;
                            xSemaphoreGive(i2cMutex);
                        }
                    }

                    xQueueSend(setTimeQueue, &setTime, 0);
                }
            }
        }
    }
}

void getICM42670(void *pvParameters)
{
    // Initialize device descriptor and device
    icm42670_t dev = {0};
    ESP_ERROR_CHECK(icm42670_init_desc(&dev, ICM42670_I2C_ADDRESS, PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // Configure interrupt pin
    const uint8_t int_pin = 1;
    const icm42670_int_config_t int_config = {
        .mode = ICM42670_INT_MODE_LATCHED,
        .drive = ICM42670_INT_DRIVE_PUSH_PULL,
        .polarity = ICM42670_INT_POLARITY_ACTIVE_HIGH,
    };
    ESP_ERROR_CHECK(icm42670_config_int_pin(&dev, int_pin, int_config));

    // Set interrupt sources
    icm42670_int_source_t sources = {false};
    sources.wom_y = true; // Only using y-axis
    ESP_ERROR_CHECK(icm42670_set_int_sources(&dev, int_pin, sources));

    // Configure wake-on-motion
    const icm42670_wom_config_t wom_config = {
        .trigger = ICM42670_WOM_INT_DUR_FIRST,
        .logical_mode = ICM42670_WOM_INT_MODE_ALL_OR,
        .reference = ICM42670_WOM_MODE_REF_INITIAL,
        .wom_y_threshold = 255, // Only using y-axis
    };
    ESP_ERROR_CHECK(icm42670_config_wom(&dev, wom_config));

    // Set power modes and other configurations
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_accel_avg(&dev, ICM42670_ACCEL_AVG_8X));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_4G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_500DPS));
    ESP_ERROR_CHECK(icm42670_enable_wom(&dev, true));

    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG_ICM42670, "Initial temperature: %.2f°C", temperature);

    while (1)
    {
    	uint32_t ulNotificationValue;
    	if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
    	{
			if (ulNotificationValue == 1)
			{
		        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
		        {
		            // Read the interrupt status to clear the interrupt
		            uint8_t int_status;
		            ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, ICM42670_REG_INT_STATUS, (int16_t*)&int_status));

		            // Read the data
	                icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_X1, &imuData.accel_x);
	                icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Y1, &imuData.accel_y);
	                icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Z1, &imuData.accel_z);
	                icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_X1, &imuData.gyro_x);
	                icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Y1, &imuData.gyro_y);
	                icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Z1, &imuData.gyro_z);

	                if (convert_to_degrees(imuData.accel_y) > -35 && !backlightOn)
	                {
	                    gpio_set_level(BACKLIGHT_PIN, 1); // Turn display on
	                    backlightOn = true;
	                }
	                else if (convert_to_degrees(imuData.accel_y) <= -35 && currentScreen == 0 && sleepTimerOn && !laserOn && !lidarOn)
	                {
	                	enter_light_sleep();
	                }

	                if (currentScreen == 1) // If IMU screen
	                {
						if (xQueueSend(ICMQueue, &imuData, portMAX_DELAY) != pdPASS)
						{
							ESP_LOGE(TAG_ICM42670, "Failed to send data to queue");
						}
	                }

	                xSemaphoreGive(i2cMutex);
		        }
			}
    	}

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void getMCP3427(void *arg)
{
    // Clear device descriptor
    memset(&adc, 0, sizeof(adc));

    ESP_ERROR_CHECK(mcp342x_init_desc(&adc, MCP3427_I2C_ADDRESS, 0, 13, 14));

    adc.channel = CHANNEL;
    adc.gain = GAIN;
    adc.resolution = RESOLUTION;
    adc.mode = MCP342X_CONTINUOUS;

    uint32_t wait_time;
    ESP_ERROR_CHECK(mcp342x_get_sample_time_us(&adc, &wait_time)); // microseconds
    wait_time = wait_time / 1000 + 1; // milliseconds

    // start first conversion
    ESP_ERROR_CHECK(mcp342x_start_conversion(&adc));

    while (1)
    {
    	uint32_t ulNotificationValue;
    	if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
    	{
			if (ulNotificationValue == 1)
			{
				if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
				{
			        // Wait for conversion
			        vTaskDelay(pdMS_TO_TICKS(wait_time));

			        // Read data
			        float volts;
			        ESP_ERROR_CHECK(mcp342x_get_voltage(&adc, &volts, NULL));
			        //printf("Channel: %d, voltage: %0.4f\n", adc.channel, volts);

					// Send the structured data to the queue
					if (xQueueSend(MCPQueue, &volts, portMAX_DELAY) != pdPASS)
					{
						ESP_LOGE(TAG_ICM42670, "Failed to send data to queue");
					}
					xSemaphoreGive(i2cMutex);
				}
				vTaskDelay(pdMS_TO_TICKS(500));
			}
    	}
    }
}

void sendWireless(void *pvParameters)
{
	volatile bool canSend = false;

    for (int i = 0; i < MAX_MAC_ADDRESSES; i++) {
        for (int j = 0; j < MAC_ADDRESS_LENGTH; j++) {
            macaddresses[i][j] = 0x0;
        }
    }

    while (1)
    {
        uint32_t ulNotificationValue;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &ulNotificationValue, portMAX_DELAY) == pdTRUE)
        {
            if (ulNotificationValue == 1)
            {
                vTaskDelay(pdMS_TO_TICKS(150)); // Main loop delay to reduce CPU usage

                if (currentScreen == 2)
                {
                	int r = selectedMAC - 1;
					if (gpio_get_level(BUTTON_4) == 1)
					{
						switch (selectedDigit)
						{
							case 1: macaddresses[r][0] = (macaddresses[r][0] & 0x0F) | (((macaddresses[r][0] >> 4) + 1) << 4); if ((macaddresses[r][0] >> 4) > 0xF) macaddresses[r][0] &= 0x0F; break;
							case 2: macaddresses[r][0] = (macaddresses[r][0] & 0xF0) | ((macaddresses[r][0] + 1) & 0x0F); if ((macaddresses[r][0] & 0x0F) > 0xF) macaddresses[r][0] &= 0xF0; break;
							case 3: macaddresses[r][1] = (macaddresses[r][1] & 0x0F) | (((macaddresses[r][1] >> 4) + 1) << 4); if ((macaddresses[r][1] >> 4) > 0xF) macaddresses[r][1] &= 0x0F; break;
							case 4: macaddresses[r][1] = (macaddresses[r][1] & 0xF0) | ((macaddresses[r][1] + 1) & 0x0F); if ((macaddresses[r][1] & 0x0F) > 0xF) macaddresses[r][1] &= 0xF0; break;
							case 5: macaddresses[r][2] = (macaddresses[r][2] & 0x0F) | (((macaddresses[r][2] >> 4) + 1) << 4); if ((macaddresses[r][2] >> 4) > 0xF) macaddresses[r][2] &= 0x0F; break;
							case 6: macaddresses[r][2] = (macaddresses[r][2] & 0xF0) | ((macaddresses[r][2] + 1) & 0x0F); if ((macaddresses[r][2] & 0x0F) > 0xF) macaddresses[r][2] &= 0xF0; break;
							case 7: macaddresses[r][3] = (macaddresses[r][3] & 0x0F) | (((macaddresses[r][3] >> 4) + 1) << 4); if ((macaddresses[r][3] >> 4) > 0xF) macaddresses[r][3] &= 0x0F; break;
							case 8: macaddresses[r][3] = (macaddresses[r][3] & 0xF0) | ((macaddresses[r][3] + 1) & 0x0F); if ((macaddresses[r][3] & 0x0F) > 0xF) macaddresses[r][3] &= 0xF0; break;
							case 9: macaddresses[r][4] = (macaddresses[r][4] & 0x0F) | (((macaddresses[r][4] >> 4) + 1) << 4); if ((macaddresses[r][4] >> 4) > 0xF) macaddresses[r][4] &= 0x0F; break;
							case 10: macaddresses[r][4] = (macaddresses[r][4] & 0xF0) | ((macaddresses[r][4] + 1) & 0x0F); if ((macaddresses[r][4] & 0x0F) > 0xF) macaddresses[r][4] &= 0xF0; break;
							case 11: macaddresses[r][5] = (macaddresses[r][5] & 0x0F) | (((macaddresses[r][5] >> 4) + 1) << 4); if ((macaddresses[r][5] >> 4) > 0xF) macaddresses[r][5] &= 0x0F; break;
							case 12: macaddresses[r][5] = (macaddresses[r][5] & 0xF0) | ((macaddresses[r][5] + 1) & 0x0F); if ((macaddresses[r][5] & 0x0F) > 0xF) macaddresses[r][5] &= 0xF0; break;
						}
					}


                    if (gpio_get_level(BUTTON_5) == 1)
                    {
                        selectedDigit++;
                        if (selectedDigit > 12) {
                            selectedDigit = 1;
                        }
                    }

                    if (gpio_get_level(BUTTON_3) == 1)
                    {
                        selectedMAC++;
                        if (selectedMAC > MAX_MAC_ADDRESSES) {
                        	selectedMAC = 1;
                        }
                    }

					if (xQueueSend(wirelessQueue, &macaddresses, portMAX_DELAY) != pdPASS)
					{
						ESP_LOGE(TAG_ICM42670, "Failed to send data to queue");
					}

					if (gpio_get_level(BUTTON_2) == 1 && canSend)
					{
						init_wireless();
					    ESP_ERROR_CHECK(esp_now_init()); // When using ESP-NOW
					    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent)); // When using ESP-NOW
						add_peer(macaddresses[selectedMAC - 1]);
						uint8_t data[] = {1}; // An array of uint8_t
						esp_now_send(macaddresses[selectedMAC - 1], data, sizeof(data));
					    ESP_ERROR_CHECK(esp_now_del_peer(macaddresses[selectedMAC - 1]));
					    ESP_ERROR_CHECK(esp_now_deinit());

					    deinit_wireless();
					    canSend = false;
					}
					else if (gpio_get_level(BUTTON_2) == 0)
					{
						canSend = true;
					}
                }
            }
        }
    }
}

void ST7789(void *pvParameters)
{
	// Set fonts
	FontxFile fx16G[2];
	FontxFile fx24G[2];
	FontxFile fx32G[2];
	FontxFile fx32L[2];
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/ILGH32XB.FNT",""); // 16x32Dot Gothic
	InitFontx(fx32L,"/spiffs/LATIN32B.FNT",""); // 16x32Dot Latin

	FontxFile fx16M[2];
	FontxFile fx24M[2];
	FontxFile fx32M[2];
	InitFontx(fx16M,"/spiffs/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/ILMH32XB.FNT",""); // 16x32Dot Mincyo

	TFT_t dev;
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

	char file[32];

	//FillCobalt(&dev, CONFIG_WIDTH, CONFIG_HEIGHT);

	// Initial logo on boot
	strcpy(file, "/spiffs/roboticworx.png");
	drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
	reset_sleep_timer();
	vTaskDelay(pdMS_TO_TICKS(500));
	reset_sleep_timer();

	//bool lidarActive = false; // Toggle this
	//bool canDraw = true;
	//bool eraseOnce = true;

	// Multi Font Test
	uint16_t color;
	uint8_t ascii[40];
	uint16_t xpos = 0;
	uint16_t ypos = 50;

	int themeSelected = 2; // CHANGE TO SELECT DEFAULT WATCH THEME (1 = Steam-punk, 2 = Planets). Feel free to also add your own themes. Instructions on roboticworx.io.

	bool screen_face = true;
	bool screen_imu = false;
	bool screen_wireless = false;
	bool screen_time = false;
	bool screen_flashlight = false;
	bool screen_wifi = false;
	bool screen_docs = false;

	bool hour24 = false;
	bool updateMACs = true;

	bool canSend1 = true;
	bool canSend2 = true;
	bool canSend3 = true;

	volatile bool updateFreeze = true;
	bool oneLaser = true;

	bool oneLidar = true;
	bool lidarWasOn = false;
	bool lidarTriggered = false;

	// Hold the last values of the RTC to compare to. Initialize to zero to ensure first update happens
	static struct tm lastSecond = {0};
	static struct tm lastMinute = {0};
	static struct tm lastHour = {0};
	static struct tm lastDay = {0};
	static struct tm lastMonth = {0};

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(10));

		if (sleepTimerOn)
		{
			color = DESERT_ORANGE;
		}
		else
		{
			reset_sleep_timer();
			color = DESERT_WHITE;
		}

		if (gpio_get_level(BUTTON_1) == 1 || gpio_get_level(BUTTON_2) == 1 || gpio_get_level(BUTTON_3) == 1 || gpio_get_level(BUTTON_4) == 1 || gpio_get_level(BUTTON_5) == 1)
		    reset_sleep_timer();

		if (gpio_get_level(BUTTON_3) == 0)
			oneLaser = true;
		if (gpio_get_level(BUTTON_2) == 0)
			oneLidar = true;

		if (laserOn)
		{
			gpio_set_level(LASER_PIN, 1);
			reset_sleep_timer();
		}
		else
			gpio_set_level(LASER_PIN, 0);

		// Main screens check. Start with doubles
		if (gpio_get_level(BUTTON_1) == 1 && gpio_get_level(BUTTON_2) == 1 && gpio_get_level(BUTTON_3) == 1) // QR
			currentScreen = 11;
		else if (gpio_get_level(BUTTON_4) == 1 && gpio_get_level(BUTTON_3) == 1 && currentScreen != 2) // Clock
			currentScreen = 3;
		else if (gpio_get_level(BUTTON_4) == 1 && gpio_get_level(BUTTON_2) == 1 && currentScreen != 2) // Networks
			currentScreen = 4;
		else if (gpio_get_level(BUTTON_4) == 1 && gpio_get_level(BUTTON_1) == 1 && currentScreen != 2) // Flashlight
			currentScreen = 10;
		else if (gpio_get_level(BUTTON_1) == 1 && gpio_get_level(BUTTON_3) == 1 && currentScreen == 0 && oneLaser)
		{
			laserOn = !laserOn;
			oneLaser = false;
		}
		else if (gpio_get_level(BUTTON_1) == 1 && gpio_get_level(BUTTON_2) == 1 && currentScreen == 0 && oneLidar)
		{
			lidarOn = !lidarOn; // MAKE DISABLE LIDAR TURN OFF ENABLE GPIO
			oneLidar = false;
			lidarTriggered = true;
			//if (lidarWasOn)
				//screen_face = true;
		}
		else if (gpio_get_level(BUTTON_1) == 1 && gpio_get_level(BUTTON_5) == 1 && canSend1 && currentScreen != 2) // Hotkey 1
		{
			init_wireless();
		    ESP_ERROR_CHECK(esp_now_init()); // When using ESP-NOW
		    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent)); // When using ESP-NOW
			add_peer(macaddresses[0]);
			uint8_t data[] = {1}; // An array of uint8_t
			esp_now_send(macaddresses[0], data, sizeof(data));
		    ESP_ERROR_CHECK(esp_now_del_peer(macaddresses[0]));
		    ESP_ERROR_CHECK(esp_now_deinit());

		    deinit_wireless();
		    canSend1 = false;
		}
		else if (gpio_get_level(BUTTON_2) == 1 && gpio_get_level(BUTTON_5) == 1 && canSend2 && currentScreen != 2) // Hotkey 2
		{
			init_wireless();
		    ESP_ERROR_CHECK(esp_now_init()); // When using ESP-NOW
		    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent)); // When using ESP-NOW
			add_peer(macaddresses[1]);
			uint8_t data[] = {1}; // An array of uint8_t
			esp_now_send(macaddresses[1], data, sizeof(data));
		    ESP_ERROR_CHECK(esp_now_del_peer(macaddresses[1]));
		    ESP_ERROR_CHECK(esp_now_deinit());

		    deinit_wireless();
		    canSend2 = false;
		}
		else if (gpio_get_level(BUTTON_3) == 1 && gpio_get_level(BUTTON_5) == 1 && canSend3 && currentScreen != 2) // Hotkey 3
		{
			init_wireless();
		    ESP_ERROR_CHECK(esp_now_init()); // When using ESP-NOW
		    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent)); // When using ESP-NOW
			add_peer(macaddresses[2]);
			uint8_t data[] = {1}; // An array of uint8_t
			esp_now_send(macaddresses[2], data, sizeof(data));
		    ESP_ERROR_CHECK(esp_now_del_peer(macaddresses[2]));
		    ESP_ERROR_CHECK(esp_now_deinit());

		    deinit_wireless();
		    canSend3 = false;
		}
		else if (gpio_get_level(BUTTON_1) == 1 && currentScreen != 0) // Main face
		{
			currentScreen = 0;
			updateTime = true;
		}
		else if (gpio_get_level(BUTTON_4) == 1 && gpio_get_level(BUTTON_5) == 1 && currentScreen == 0 && updateFreeze) // Freeze main face
		{
			sleepTimerOn = !sleepTimerOn;
			updateFreeze = false;
		}
		else if (gpio_get_level(BUTTON_3) == 1 && gpio_get_level(BUTTON_4) == 0 && currentScreen != 2 && gpio_get_level(BUTTON_5) == 0 && gpio_get_level(BUTTON_1) == 0) // IMU
			currentScreen = 1;
		else if (gpio_get_level(BUTTON_2) == 1 && gpio_get_level(BUTTON_4) == 0 && currentScreen != 2 && gpio_get_level(BUTTON_5) == 0 && gpio_get_level(BUTTON_1) == 0) // Wireless
			currentScreen = 2;

		if (gpio_get_level(BUTTON_4) == 0 && gpio_get_level(BUTTON_5) == 0 && currentScreen == 0)
			updateFreeze = true;
		if (gpio_get_level(BUTTON_1) == 0)
			canSend1 = true;
		if (gpio_get_level(BUTTON_2) == 0)
			canSend2 = true;
		if (gpio_get_level(BUTTON_3) == 0)
			canSend3 = true;

		if (currentScreen == 0)
		{
			if (screen_face)
			{
				if (themeSelected == 1)
					strcpy(file, "/spiffs/face1.png");
				else if (themeSelected == 2)
					strcpy(file, "/spiffs/face2.png");

				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_imu = true;
				screen_wireless = true;
				screen_time = true;
				screen_flashlight = true;
				screen_wifi = true;
				screen_docs = true;
				screen_face = false;

				last_wakeup = xTaskGetTickCount();

			}

			if (lidarOn)
			{
				xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
				xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
				xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
				xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
				xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
				xTaskNotifyGive(getVL53L1XHandle);
				xTaskNotify(getVL53L1XHandle, 1, eSetValueWithOverwrite);
				reset_sleep_timer();
				last_wakeup = xTaskGetTickCount();
				lidarWasOn = false;
			}
			else
			{
				lidarWasOn = true;
				xTaskNotify(getMCP3427Handle, 1, eSetValueWithOverwrite);
				xTaskNotify(getBME680Handle, 1, eSetValueWithOverwrite);
				xTaskNotify(getDS3231Handle, 1, eSetValueWithOverwrite);
				xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
				xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
				xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);
			}

			lcdSetFontDirection(&dev, 0);
			lcdSetFontFill(&dev, BLACK);

			bme680_values_float_t values;
			if (xQueueReceive(bmeQueue, &values, 0)) //
			{
				if (themeSelected == 1)
				{
					xpos = 30;
					ypos = 102;
				}
				else if (themeSelected == 2)
				{
					xpos = 23;
					ypos = 262;
				}

				float temp = values.temperature; // Offset due to other components
				sprintf((char *)ascii, "%.2fC", temp); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string

				if (themeSelected == 1)
				{
					xpos = 30;
					ypos = 83;
				}
				else if (themeSelected == 2)
				{
					xpos = 23;
					ypos = 243;
				}

				float humidity = values.humidity; // Example integer
				sprintf((char *)ascii, "%.2f%%", humidity); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string

				if (themeSelected == 1)
				{
					xpos = 30;
					ypos = 45;
				}
				else if (themeSelected == 2)
				{
					xpos = 82;
					ypos = 240;
				}

				float pressure = values.pressure; // Example integer
				sprintf((char *)ascii, "%.2fhPa", pressure); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string

				if (themeSelected == 1)
				{
					xpos = 6;
					ypos = 242;
				}
				else if (themeSelected == 2)
				{
					xpos = 82;
					ypos = 264;
				}

				float gas_resistance = values.gas_resistance; // Example integer
				// float comp_gas = log(gas_resistance) + 0.04 * gas_resistance; // comp_gas = log(R_gas[ohm]) + 0.04 log(Ohm)/%rh * hum[%rh]

				gas_resistance /= 1000;
				if (gas_resistance < 100)
				{
					sprintf((char *)ascii, "0%.2f", gas_resistance); // Convert integer to string
					lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color); // Display the integer as a string
				}
				else
				{
					sprintf((char *)ascii, "%.2f", gas_resistance); // Convert integer to string
					lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color); // Display the integer as a string
				}

				if (themeSelected == 1)
				{
					xpos = 30;
					ypos = 64;
				}
				else if (themeSelected == 2)
				{
					xpos = 23;
					ypos = 224;
				}

				float altitude = 44330 * (1 - pow((values.pressure/1013.25), (1/5.255)));
				sprintf((char *)ascii, "%.2fm ", altitude); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string
			}

			float volts;
			if (xQueueReceive(MCPQueue, &volts, 0))
			{
				if (themeSelected == 1)
				{
					xpos = 162;
					ypos = 242;
					float percentage = convert_voltage_to_percentage(volts);

					if (percentage == 100)
					{
						sprintf((char *)ascii, "%.2f", percentage); // Convert integer to string
						color = DESERT_GREEN;
					}
					else
					{
						if (percentage >= 60 || gpio_get_level(CHARGE_PIN) == 0)// If IS charging or high battery
							color = DESERT_GREEN;
						else if (percentage < 60 && percentage > 20)
							color = DESERT_ORANGE;
						else
							color = DESERT_RED;
						sprintf((char *)ascii, "%.2f%%", percentage); // Convert integer to string
					}
					lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color); // Display the integer as a string

				}
				else if (themeSelected == 2)
				{
					xpos = 170;
					ypos = 35;
					float percentage = convert_voltage_to_percentage(volts);

					if (percentage == 100)
					{
						sprintf((char *)ascii, "%.2f", percentage); // Convert integer to string
						color = DESERT_GREEN;
					}
					else
					{
						if (percentage >= 60 || gpio_get_level(CHARGE_PIN) == 0)// If IS charging or high battery
							color = DESERT_GREEN;
						else if (percentage < 60 && percentage > 20)
							color = DESERT_ORANGE;
						else
							color = DESERT_RED;
						sprintf((char *)ascii, "%.2f%%", percentage); // Convert integer to string
					}
					lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string
				}

				if (themeSelected == 1)
				{
					xpos = 176;
					ypos = 262;
				}
				else if (themeSelected == 2)
				{
					xpos = 170;
					ypos = 50;
				}
				float bat_voltage = convert_voltage_bat(volts);
				sprintf((char *)ascii, "%.2fV", bat_voltage); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the integer as a string
			}

			// Real Time Clock (RTC)
			//  time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, temp);
			struct tm currentTime;
			if (xQueueReceive(timeQueue, &currentTime, 0))
			{

				// Check if time data has changed
				color = DESERT_BLUE;

				if (currentTime.tm_sec != lastSecond.tm_sec || updateTime)
				{
					if (themeSelected == 1)
					{
						xpos = 173;
						ypos = 93;
					}
					else if (themeSelected == 2)
					{
						xpos = 120;
						ypos = 50;
					}

			        // Clear the buffer before use
			        memset(ascii, 0, sizeof(ascii));

					if (currentTime.tm_sec < 10)
					{
						sprintf((char *)ascii, "0%d ", currentTime.tm_sec);
						lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
					}
					else
					{
						sprintf((char *)ascii, "%d ", currentTime.tm_sec);
						lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
					}

					memcpy(&lastSecond, &currentTime, sizeof(struct tm)); // Update lastTime with the current time after updating the display
				}

				if (currentTime.tm_min != lastMinute.tm_min || updateTime)
				{
					if (themeSelected == 1)
					{
						xpos = 167;
						ypos = 70;
					}
					else if (themeSelected == 2)
					{
						xpos = 72;
						ypos = 52;
					}

			        // Clear the buffer before use
			        memset(ascii, 0, sizeof(ascii));


					if (currentTime.tm_min < 10)
					{
						sprintf((char *)ascii, "0%d ", currentTime.tm_min);
						lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
					}
					else
					{
						sprintf((char *)ascii, "%d ", currentTime.tm_min);
						lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
					}

					memcpy(&lastMinute, &currentTime, sizeof(struct tm)); // Update lastTime with the current time after updating the display
				}

				if (currentTime.tm_hour != lastHour.tm_hour || updateTime)
				{

					if (themeSelected == 1)
					{
						xpos = 119;
						ypos = 70;
					}
					else if (themeSelected == 2)
					{
						xpos = 25;
						ypos = 52;
					}

			        // Clear the buffer before use
			        memset(ascii, 0, sizeof(ascii));

					if (!hour24)
					{
						int hour = currentTime.tm_hour;
						if (hour > 12)
							hour -= 12;
						else if (hour == 0)
							hour = 12;
						else
							hour = currentTime.tm_hour;

						if (hour < 10)
						{
							sprintf((char *)ascii, "0%d:", hour);
							lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
						}
						else
						{
							sprintf((char *)ascii, "%d:", hour);
							lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
						}
					}
					else
					{
						if (currentTime.tm_hour < 10)
						{
							sprintf((char *)ascii, "0%d:", currentTime.tm_hour);
							lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
						}
						else
						{
							sprintf((char *)ascii, "%d:", currentTime.tm_hour);
							lcdDrawString(&dev, fx32M, xpos, ypos, ascii, color);
						}
					}

					memcpy(&lastHour, &currentTime, sizeof(struct tm)); // Update lastTime with the current time after updating the display
				}

				if (currentTime.tm_mday != lastDay.tm_mday || updateTime)
				{
					if (themeSelected == 1)
					{
						xpos = 169;
						ypos = 160;
					}
					else if (themeSelected == 2)
					{
						xpos = 75;
						ypos = 78;
					}

			        // Clear the buffer before use
			        memset(ascii, 0, sizeof(ascii));

					if (currentTime.tm_mday < 10)
					{
						sprintf((char *)ascii, "0%d ", currentTime.tm_mday);
						lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
					}
					else
					{
						sprintf((char *)ascii, "%d ", currentTime.tm_mday);
						lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
					}

					memcpy(&lastDay, &currentTime, sizeof(struct tm)); // Update lastTime with the current time after updating the display
				}

				if (currentTime.tm_mon != lastMonth.tm_mon || updateTime)
				{
			        // Clear the buffer before use
			        memset(ascii, 0, sizeof(ascii));

					if (themeSelected == 1)
					{
						xpos = 45;
						ypos = 160;
						if (currentTime.tm_mon < 9)
						{
							sprintf((char *)ascii, "0%d", currentTime.tm_mon + 1);
							lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
						}
						else
						{
							sprintf((char *)ascii, "%d", currentTime.tm_mon + 1);
							lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
						}
					}
					else if (themeSelected == 2)
					{
						xpos = 27;
						ypos = 78;
						if (currentTime.tm_mon < 9)
						{
							sprintf((char *)ascii, "0%d", currentTime.tm_mon + 1);
							lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
						}
						else
						{
							sprintf((char *)ascii, "%d", currentTime.tm_mon + 1);
							lcdDrawString(&dev, fx24M, xpos, ypos, ascii, color);
						}
					}

					memcpy(&lastMonth, &currentTime, sizeof(struct tm)); // Update lastTime with the current time after updating the display
				}
				updateTime = false;
			}

			// LiDAR
			uint16_t distance;
			if (xQueueReceive(lidarQueue, &distance, 0)) //  && lidarActive
			{
				color = DESERT_RED;


				if (themeSelected == 1)
				{
					xpos = 84;
					ypos = 134;
				}
				else if (themeSelected == 2)
				{
					xpos = 140;
					ypos = 164;
				}

				float centis = distance;
				centis /= 10;
				sprintf((char *)ascii, "%.2f", centis); // Format as integer
				// If you really need to format as float:
				// sprintf((char *)ascii, "%.2f", (float)distance);
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
				{
					xpos = 107;
					ypos = 153;
				}
				else if (themeSelected == 2)
				{
					xpos = 175;
					ypos = 185;
				}

				sprintf((char *)ascii, "cm"); // Convert integer to string
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color); // Display the integer as a string

				if (lidarWasOn && lidarTriggered)
				{
					screen_face = true;
					updateTime = true;
					lidarTriggered = false;
				}

			}
		}

		else if (currentScreen == 1)
		{

			if (screen_imu)
			{
				if (themeSelected == 1)
					strcpy(file, "/spiffs/face1_imu.png");
				else if (themeSelected == 2)
					strcpy(file, "/spiffs/face2_imu.png");

				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_wireless = true;
				screen_time = true;
				screen_flashlight = true;
				screen_wifi = true;
				screen_docs = true;
				screen_imu = false;

			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);

			// ICM42670
			icm42670_data_t imuData;
			if (xQueueReceive(ICMQueue, &imuData, 0)) //  && lidarActive
			{
				color = DESERT_BLUE;

				if (themeSelected == 1)
				{
					xpos = 88;
					ypos = 90;
				}
				else if (themeSelected == 2)
				{
					xpos = 120;
					ypos = 174;
				}

				sprintf((char *)ascii, "%f    ", convert_to_degrees(imuData.accel_x)); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
					ypos = 109;
				else if (themeSelected == 2)
					ypos = 192;

				sprintf((char *)ascii, "%f    ", convert_to_degrees(imuData.accel_y)); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
					ypos = 127;
				else if (themeSelected == 2)
					ypos = 210;

				sprintf((char *)ascii, "%f    ", convert_to_degrees(imuData.accel_z)); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
					ypos = 161;
				else if (themeSelected == 2)
					ypos = 235;

				sprintf((char *)ascii, "%d    ", imuData.gyro_x); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
					ypos = 180;
				else if (themeSelected == 2)
					ypos = 253;

				sprintf((char *)ascii, "%d    ", imuData.gyro_y); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string

				if (themeSelected == 1)
					ypos = 198;
				else if (themeSelected == 2)
					ypos = 271;

				sprintf((char *)ascii, "%d    ", imuData.gyro_z); // Convert integer to string
				lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color); // Display the distance as a string
			}
		}

		else if (currentScreen == 2) // Wireless screen
		{

			if (screen_wireless)
			{
				if (themeSelected == 1)
					strcpy(file, "/spiffs/face1_wireless.png");
				else if (themeSelected == 2)
					strcpy(file, "/spiffs/face2_wireless.png");

				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_imu = true;
				screen_flashlight = true;
				screen_time = true;
				screen_wifi = true;
				screen_docs = true;
				screen_wireless = false;
				updateMACs = true;
				selectedMAC = 4;
				selectedDigit = 1;
			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 1, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);

			uint8_t macaddresses[MAX_MAC_ADDRESSES][MAC_ADDRESS_LENGTH];
			if (xQueueReceive(wirelessQueue, &macaddresses, 0))
			{
			    uint16_t selectedColor = DESERT_ORANGE;
			    uint16_t defaultColor = DESERT_BLUE;

			    void draw_mac_address(uint8_t mac[], int y_offset, uint8_t selectedDigit)
			    {
			        uint16_t xpos = 44;
			        char ascii[3];
			        for (int i = 0; i < 6; i++)
			        {
			            sprintf(ascii, "%02X", mac[i]);
			            for (int j = 0; j < 2; j++)
			            {
			                uint16_t color = ((i * 2 + j + 1) == selectedDigit) ? selectedColor : defaultColor;
			                char digitStr[2] = {ascii[j], '\0'};
			                lcdDrawString(&dev, fx16G, xpos, y_offset, (unsigned char *)digitStr, color);
			                xpos += 9; // Adjust according to the font width
			            }
			            if (i < 5)
			            {
			                char colon = ':';
			                lcdDrawString(&dev, fx16G, xpos, y_offset, (unsigned char *)&colon, defaultColor);
			                xpos += 9; // Adjust according to the colon width
			            }
			        }
			    }

			    void draw_brace1(int ypos, uint16_t color)
			    {
			        uint16_t xpos = 35;
			        char brace1[] = "{";
			        lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)brace1, color);
			    }
			    void draw_brace2(int ypos, uint16_t color)
			    {
			        uint16_t xpos = 197;
			        char brace2[] = "}";
			        lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)brace2, color);
			    }

			    int y_offsets[] = {95, 110, 125, 140, 155, 170, 185, 200, 215, 230, 245};
			    int y_offsets2[] = {63, 83, 103, 123, 143, 163, 183, 203, 223, 243, 263};
			    if (updateMACs) // Do once when first opening screen
			    {
				    for (int i = 0; i < 11; i++)
				    {
				            if (themeSelected == 1)
				            {
				            	if (selectedMAC == i + 1)
				            	{
				            		draw_brace1(y_offsets[i], DESERT_WHITE);
				            		draw_brace2(y_offsets[i], DESERT_WHITE);
				            	}
				                draw_mac_address(macaddresses[i], y_offsets[i], selectedDigit);
				            }
				            else if (themeSelected == 2)
				            {
				            	if (selectedMAC == i + 1)
				            	{
				            		draw_brace1(y_offsets2[i], DESERT_WHITE);
				            		draw_brace2(y_offsets2[i], DESERT_WHITE);
				            	}
				                draw_mac_address(macaddresses[i], y_offsets2[i], selectedDigit);
				            }
				        else
				        {
				        	if (themeSelected == 1)
				        	{
				        		draw_brace1(y_offsets[i], DESERT_BLACK);
				        		draw_brace2(y_offsets[i], DESERT_BLACK);
				        	}
				        	else if (themeSelected == 2)
				        	{
				        		draw_brace1(y_offsets2[i], DESERT_BLACK);
				        		draw_brace2(y_offsets2[i], DESERT_BLACK);
				        	}
				        }
				    }
			    }
			    for (int i = 0; i < 11; i++)
			    {
			        if (selectedMAC == i + 1)
			        {
			            if (themeSelected == 1)
			            {
			            	if (selectedMAC == i + 1)
			            	{
			            		draw_brace1(y_offsets[i], DESERT_WHITE);
			            		draw_brace2(y_offsets[i], DESERT_WHITE);
			            	}
			                draw_mac_address(macaddresses[selectedMAC - 1], y_offsets[i], selectedDigit);

			            }
			            else if (themeSelected == 2)
			            {
			            	if (selectedMAC == i + 1)
			            	{
			            		draw_brace1(y_offsets2[i], DESERT_WHITE);
			            		draw_brace2(y_offsets2[i], DESERT_WHITE);
			            	}
			                draw_mac_address(macaddresses[selectedMAC - 1], y_offsets2[i], selectedDigit);
			            }
			        }
			        else
			        {
			        	if (themeSelected == 1)
			        	{
			        		draw_brace1(y_offsets[i], DESERT_BLACK);
			        		draw_brace2(y_offsets[i], DESERT_BLACK);
			        	}
			        	else if (themeSelected == 2)
			        	{
			        		draw_brace1(y_offsets2[i], DESERT_BLACK);
			        		draw_brace2(y_offsets2[i], DESERT_BLACK);
			        	}
			        }
			    }
			    updateMACs = false;
			}
		}
		else if (currentScreen == 3) // Change time
		{
			if (screen_time)
			{
				if (themeSelected == 1)
					strcpy(file, "/spiffs/face1_clock.png");
				else if (themeSelected == 2)
					strcpy(file, "/spiffs/face2_clock.png");

				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_imu = true;
				screen_flashlight = true;
				screen_wireless = true;
				screen_wifi = true;
				screen_docs = true;
				screen_time = false;

			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);

			struct tm newTime;
			if (xQueueReceive(setTimeQueue, &newTime, 0))
			{
				if (themeSelected == 1)
				{
					xpos = 120;
					ypos = 96;
				}
				else if (themeSelected == 2)
				{
					xpos = 110;
					ypos = 82;
				}
				if (selectedTime == 1)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;

				sprintf((char *)ascii, "%d  ", newTime.tm_year + 1900); // tm_year is years since 1900
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);

				if (themeSelected == 1)
					ypos = 128;
				else if (themeSelected == 2)
					ypos = 114;
				if (selectedTime == 2)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;
				sprintf((char *)ascii, "%d  ", newTime.tm_mon + 1); // 0 Based
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);

				if (themeSelected == 1)
					ypos = 159;
				else if (themeSelected == 2)
					ypos = 145;
				if (selectedTime == 3)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;
				sprintf((char *)ascii, "%d  ", newTime.tm_mday);
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);

				if (themeSelected == 1)
					ypos = 190;
				else if (themeSelected == 2)
					ypos = 175;
				if (selectedTime == 4)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;
				sprintf((char *)ascii, "%d  ", newTime.tm_hour);
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);

				if (themeSelected == 1)
					ypos = 220;
				else if (themeSelected == 2)
					ypos = 205;
				if (selectedTime == 5)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;
				sprintf((char *)ascii, "%d  ", newTime.tm_min);
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);

				if (themeSelected == 1)
					ypos = 250;
				else if (themeSelected == 2)
					ypos = 235;
				if (selectedTime == 6)
					color = DESERT_BLUE;
				else
					color = DESERT_ORANGE;
				sprintf((char *)ascii, "%d  ", newTime.tm_sec);
				lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);
			}

		}
		else if (currentScreen == 4) // WiFi screen
		{
			if (screen_wifi)
			{
				if (themeSelected == 1)
					strcpy(file, "/spiffs/face1_blank.png");
				else if (themeSelected == 2)
					strcpy(file, "/spiffs/face2_blank.png");

				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_imu = true;
				screen_flashlight = true;
				screen_time = true;
				screen_wireless = true;
				screen_docs = true;
				screen_wifi = false;

				init_wireless();

			    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
			    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
			    uint16_t ap_count = 0;
			    memset(ap_info, 0, sizeof(ap_info));

			    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
			    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
			    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
			    //ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);

			    char ascii[128]; // Ensure the buffer is large enough

			    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++)
			    {
			        //ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
			        //ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi); // decibels relative to one milliwatt (higher the better)
			        //ESP_LOGI(TAG, "Channel \t\t%d", ap_info[i].primary);
			        //ESP_LOGI(TAG, "Auth Mode \t\t%d", ap_info[i].authmode);
					if (i == 0 && themeSelected == 1)
					{
					    color = DESERT_BLUE;
						xpos = 33;
						ypos = 43;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d  ", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d  ", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}
					else if (i == 0 && themeSelected == 2)
					{
					    color = DESERT_BLUE;
						xpos = 13;
						ypos = 43;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d  ", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d  ", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}

					if (i == 1 && themeSelected == 1)
					{
						color = DESERT_GREEN;
						ypos = 88;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d  ", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d  ", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}
					else if (i == 1 && themeSelected == 2)
					{
						color = DESERT_GREEN;
						ypos = 88;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d  ", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d  ", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}

					if (i == 2 && themeSelected == 1)
					{
						color = DESERT_RED;
						ypos = 133;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}
					else if (i == 2 && themeSelected == 2)
					{
						color = DESERT_RED;
						ypos = 133;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}

					if (i == 3 && themeSelected == 1)
					{
						color = DESERT_ORANGE;
						ypos = 178;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}
					else if (i == 3 && themeSelected == 2)
					{
						color = DESERT_ORANGE;
						ypos = 178;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}

					if (i == 4 && themeSelected == 1)
					{
						color = DESERT_WHITE;
						ypos = 223;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}
					else if (i == 4 && themeSelected == 2)
					{
						color = DESERT_WHITE;
						ypos = 223;
				        snprintf(ascii, sizeof(ascii), "SSID: %s", ap_info[i].ssid);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "RSSI: %d", ap_info[i].rssi);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line

					    sprintf((char *)ascii, "Auth Mode: %d", ap_info[i].authmode);
					    lcdDrawString(&dev, fx16G, xpos, ypos, (unsigned char *)ascii, color);
					    ypos += 15; // Increment y position for next line
					}

			    }

			    deinit_wireless();
			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);
		}
		else if (currentScreen == 10) // Flashlight screen
		{
			if (screen_flashlight)
			{
				strcpy(file, "/spiffs/flashlight.png");
				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_imu = true;
				screen_wireless = true;
				screen_time = true;
				screen_wifi = true;
				screen_docs = true;
				screen_flashlight = false;
			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);
		}
		else if (currentScreen == 11) // Docs screen
		{
			if (screen_docs)
			{
				strcpy(file, "/spiffs/docs.png");
				drawPNG(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
				screen_face = true;
				screen_imu = true;
				screen_wireless = true;
				screen_time = true;
				screen_wifi = true;
				screen_flashlight = true;
				screen_docs = false;
			}

			xTaskNotify(getBME680Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getDS3231Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(getICM42670Handle, 1, eSetValueWithOverwrite);
			xTaskNotify(getMCP3427Handle, 2, eSetValueWithOverwrite);
			xTaskNotify(sendWirelessHandle, 2, eSetValueWithOverwrite);
			//xTaskNotify(getVL53L1XHandle, 2, eSetValueWithOverwrite);
		}

		/*vTaskDelay(1000 / portTICK_PERIOD_MS);

		strcpy((char *)ascii, "16Dot Gothic Font");
		lcdDrawString(&dev, fx16G, xpos, ypos, ascii, color);

		xpos = 0;
		ypos = 75;
		strcpy((char *)ascii, "24Dot Gothic Font");
		lcdDrawString(&dev, fx24G, xpos, ypos, ascii, color);
		vTaskDelay(1000 / portTICK_PERIOD_MS);*/

	} // end while
}

void app_main(void)
{
	gpio_set_direction(LIDAR_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(LIDAR_PIN, 0);
	gpio_set_direction(BUTTON_1, GPIO_MODE_INPUT);
	gpio_pulldown_dis(BUTTON_1); // External pull-down already present
	gpio_pullup_dis(BUTTON_1);
	gpio_set_direction(BUTTON_2, GPIO_MODE_INPUT);
	gpio_pulldown_en(BUTTON_2);
	gpio_pullup_dis(BUTTON_2);
	gpio_set_direction(BUTTON_3, GPIO_MODE_INPUT);
	gpio_pulldown_en(BUTTON_3);
	gpio_pullup_dis(BUTTON_3);
	gpio_set_direction(BUTTON_4, GPIO_MODE_INPUT);
	gpio_pulldown_en(BUTTON_4);
	gpio_pullup_dis(BUTTON_4);
	gpio_set_direction(BUTTON_5, GPIO_MODE_INPUT);
	gpio_pulldown_en(BUTTON_5);
	gpio_pullup_dis(BUTTON_5);
	gpio_set_direction(CHARGE_PIN, GPIO_MODE_INPUT);
	gpio_pullup_en(CHARGE_PIN);
	gpio_pulldown_dis(CHARGE_PIN);
	gpio_set_direction(WOM_PIN, GPIO_MODE_INPUT);
	gpio_pulldown_en(WOM_PIN);
	gpio_pullup_dis(WOM_PIN);

	gpio_set_direction(LASER_PIN, GPIO_MODE_OUTPUT);

	ESP_LOGI(TAG_ST7789, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 12,
		.format_if_mount_failed = true
	};

	// Use settings defined above to initialize and mount SPIFFS file system.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG_ST7789, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG_ST7789, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG_ST7789, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total,&used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG_ST7789,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG_ST7789,"Partition size: total: %d, used: %d", total, used);
	}

	i2cMutex = xSemaphoreCreateMutex();
	if (i2cMutex == NULL) {
		ESP_LOGE("main", "Failed to create mutex");
	    return; // Handle error appropriately
	}

    // Create the sleep timer
    sleep_timer = xTimerCreate("SleepTimer", pdMS_TO_TICKS(SLEEP_TIMEOUT_MS), pdFALSE, (void *)0, sleep_timer_callback);
    if (sleep_timer == NULL) {
        ESP_LOGE(TAG_TIMER, "Failed to create sleep timer");
        return;
    }

    // Start the sleep timer
    if (xTimerStart(sleep_timer, 0) != pdPASS) {
        ESP_LOGE(TAG_TIMER, "Failed to start sleep timer");
        return;
    }

	bmeQueue = xQueueCreate(5, sizeof(bme680_values_float_t));
	lidarQueue = xQueueCreate(5, sizeof(uint16_t));
	timeQueue = xQueueCreate(5, sizeof(struct tm));
	ICMQueue = xQueueCreate(10, sizeof(icm42670_data_t));
	MCPQueue = xQueueCreate(5, sizeof(float));
	setTimeQueue = xQueueCreate(5, sizeof(struct tm));
	uint8_t macaddresses[MAX_MAC_ADDRESSES][MAC_ADDRESS_LENGTH];
	wirelessQueue = xQueueCreate(5, sizeof(macaddresses));

	// ST7789
	SPIFFS_Directory("/spiffs/");
	xTaskCreate(ST7789, "ST7789", 1024*6, NULL, 5, NULL);

	// Sensors
	ESP_ERROR_CHECK(i2cdev_init()); // Initialize I2C

	xTaskCreatePinnedToCore(getBME680, "getBME680", configMINIMAL_STACK_SIZE * 16, NULL, 5, &getBME680Handle, APP_CPU_NUM);
	xTaskCreate(getDS3231, "getDS3231", configMINIMAL_STACK_SIZE * 6, NULL, 5, &getDS3231Handle);
	xTaskCreatePinnedToCore(getVL53L1X, "getVL53L1X", configMINIMAL_STACK_SIZE * 16, NULL, 5, &getVL53L1XHandle, APP_CPU_NUM);
	xTaskCreatePinnedToCore(getICM42670, "getICM42670", configMINIMAL_STACK_SIZE * 16, NULL, 5, &getICM42670Handle, APP_CPU_NUM);
	xTaskCreatePinnedToCore(getMCP3427, "getMCP3427", configMINIMAL_STACK_SIZE * 8, NULL, 5, &getMCP3427Handle, APP_CPU_NUM);
	xTaskCreate(sendWireless, "sendWireless", configMINIMAL_STACK_SIZE * 6, NULL, 5, &sendWirelessHandle);
	reset_sleep_timer();
}
