/* Copyright 2021 Winfried Klum
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Licenses for components apply as stated 
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/projdefs.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "dht11.h"
#include "bmp180.h"
#include "sh1106.h"
#include "bmp280.h"
#include "openweather.h"

//#define APP_DEBUG
#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/*Special display symbols*/
#define DISCONNECTED_SYMBOL 0x0E
#define SIGNAL_HIGH_SYMBOL 0x0B
#define SIGNAL_MID_SYMBOL 0x0C
#define SIGNAL_LOW_SYMBOL 0x0D
#define ANTENNA_SYMBOL 0x0F
#define MQTT_SYMBOL 0x10

/*MQTT Topics*/
#define TOPIC_TEMP "/topic/temp"
#define TOPIC_PRESSURE "/topic/pressure"
#define TOPIC_HUMIDITY "/topic/hum"

/*MIN,MAX values for sensors*/
#define MIN_TEMP 4
#define MAX_TEMP 45
#define MIN_HUM 20
#define MAX_HUM 90
#define MIN_PRESSURE 40000
#define MAX_PRESSURE 110000
#define MIN_ALTITUDE 0
#define MAX_ALTITUDE 6000

/* Compensate altitude measurement
   using current reference pressure, preferably at the sea level,
   obtained from weather station on internet
   Assume normal air pressure at sea level of 101325 Pa
   in case weather station is not available.
 */
#define REFERENCE_PRESSURE 101325l
/* I2C Interface Pins*/
#define I2C_PIN_SDA CONFIG_I2C_PIN_SDA
#define I2C_PIN_SCL CONFIG_I2C_PIN_SCL
/* Stacksize used for Tasks*/
#define STACK_SIZE 2000

// reference pressure retrieval
#define WEATHER_DATA_RETREIVAL_PERIOD 60000
RTC_DATA_ATTR static unsigned long reference_pressure = 0l;

static struct
{
    uint32_t humidity;
    float temperature;
    uint32_t altitude;
    uint32_t pressure;
} sensor;

static struct
{
    esp_mqtt_client_handle_t client;
    uint8_t connected;
} mqtt;

static struct
{
    uint8_t connected;
    int s_retry_num;
} wifi;

static const char *TAG = "weather";
static SemaphoreHandle_t ctrl_sem1;
esp_err_t res;
bmp280_params_t params_b;
bmp280_t dev_b;

void weather_data_retreived(uint32_t *args)
{
    weather_data* weather = (weather_data*) args;

    reference_pressure = (unsigned long) (weather->pressure * 100);
    ESP_LOGI(TAG, "Openweather pressure: %lu Pa", reference_pressure);
    ESP_LOGI(TAG, "Openweather temperature: %f K", weather->temperature);
    ESP_LOGI(TAG, "Openweather humidity: %d ", weather->humidity);
}

#if (CONFIG_USE_BMP280)
void init_bme280()
{
    bmp280_init_weather_params(&params_b);

    while (bmp280_init_desc(&dev_b, BMP280_I2C_ADDRESS_1, 0, I2C_PIN_SDA, I2C_PIN_SCL) != ESP_OK)
    {
        printf("Could not init device descriptor\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    while ((res = bmp280_init(&dev_b, &params_b)) != ESP_OK)
    {
        printf("Could not init BMP280, err: %d\n", res);
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    // bool bme280p = dev.id == BME280_CHIP_ID;
    // printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
}

void bmp280_read(void *pvParamters)
{
    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(10 * 1000  / portTICK_PERIOD_MS);
        if (bmp280_force_measurement(&dev_b) != ESP_OK)
        {
            printf("Force measurement failed\n");
            continue;
        }
        if (bmp280_read_float(&dev_b, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Pres: %.2f Pa, Temp: %.2f C, Hum: %.2f%%\n", pressure, temperature, humidity);
        // printf("Pressure: %.2f Pa, Temperature: %.2f C, Humidity: %.2f%%\n", pressure, temperature, humidity);
	sensor.temperature = temperature;
	sensor.pressure = pressure;
	sensor.humidity = humidity;
    }
}
#endif

#if (CONFIG_USE_MQTT)
/*mqtt event handler*/
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        mqtt.connected = 1;
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        mqtt.connected = 0;
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
#ifdef APP_DEBUG
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
#endif
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
#ifdef APP_DEBUG
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
#endif
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

/* Standard esp event handler that calls the mqtt event handler */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
    mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt.client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt.client);
    esp_mqtt_client_start(mqtt.client);
}
#endif
/*Event Handler for Wifi events*/
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        wifi.connected = 0;
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi.connected = 0;
        if (wifi.s_retry_num < MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            wifi.s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG, "connect to the AP failed, restarting...");
            esp_restart();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        wifi.connected = 1;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi.s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/*Install I2C Bus driver*/
esp_err_t i2c_master_init(int pin_sda, int pin_scl)
{
    esp_err_t err;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = pin_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};

    err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver configuration failed with error = %d", err);
        return ESP_FAIL;
    }
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver installation failed with error = %d", err);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "I2C master driver has been installed.");
    return ESP_OK;
}

/* Initialize wifi in AP station mode with WPA */
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    s_wifi_event_group = xEventGroupCreate();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. In case your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_err_t ret = tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "temp-humid-mqtt");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to set hostname:%d", ret);
    }

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/* Get wifi signal strength symbol*/
static uint8_t signal_strength_symbol(wifi_ap_record_t *ap_info)
{
    if (ap_info->rssi < -90)
    {
        return SIGNAL_LOW_SYMBOL;
    }
    if (ap_info->rssi < -80)
    {
        return SIGNAL_MID_SYMBOL;
    }
    return SIGNAL_HIGH_SYMBOL;
}

/*mqtt write*/
static void write_mqtt_data(void)
{
    char t_buffer[10];
    char h_buffer[10];
    char p_buffer[15];
    //Configure CONFIG_MQTT_SKIP_PUBLISH_IF_DISCONNECTED through menuconfig
    //if you don't want to publish when you are disconnected
    snprintf(t_buffer, 10, "%0.1f", sensor.temperature);
    snprintf(p_buffer, 15, "%d", sensor.pressure / 100);
    itoa(sensor.humidity, h_buffer, 10);
    ESP_LOGI(TAG, "Pressure: %d hPa, Altitude: %d m, Temp: %0.1f C, Humidity: %d %%", sensor.pressure / 100, sensor.altitude, sensor.temperature, sensor.humidity);
#if (CONFIG_USE_MQTT)
    int msg_id = esp_mqtt_client_publish(mqtt.client, TOPIC_TEMP, t_buffer, 0, 1, 1);
    msg_id = esp_mqtt_client_publish(mqtt.client, TOPIC_PRESSURE, p_buffer, 0, 1, 1);
    msg_id = esp_mqtt_client_publish(mqtt.client, TOPIC_HUMIDITY, h_buffer, 0, 1, 1);
    if (msg_id < 0)
    {
        ESP_LOGE(TAG, "Unable to publish to %s", CONFIG_BROKER_URL);
    }
#endif
}

/*Write display and mqtt data*/
static void task_write_data(void *data)
{
#if (CONFIG_USE_SH1106)
    int max_buf_size = 128;
    char s_buffer[max_buf_size];
    uint8_t sig;
    esp_err_t err;
    wifi_ap_record_t ap_info;
    int disc_counter = 0;
#endif
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (xSemaphoreTake(ctrl_sem1, pdMS_TO_TICKS(10000)) == pdTRUE)
        {
#if (CONFIG_USE_SH1106)
            if (wifi.connected)
            {
                disc_counter = 0;
                err = esp_wifi_sta_get_ap_info(&ap_info);
                if (err == ESP_OK)
                {
                    sig = signal_strength_symbol(&ap_info);
                }
                else
                {
                    sig = '?';
                }
            }
            else
            {
                sig = DISCONNECTED_SYMBOL;
                if (disc_counter++ > 2)
                {
                    esp_wifi_connect();
                    disc_counter = 0;
                }
            }
	    memset(s_buffer, 0, max_buf_size*sizeof(s_buffer[0]));
	    sh1106_display_clear(NULL);
            #if CONFIG_USE_BMP180
            snprintf(s_buffer, 100, "      %c%c      %c\nTemp:    %+3.1fC\n \nHumidity:  %3d%%\n \nAltitude: %4dm\n \nPressure:%4dhP", ANTENNA_SYMBOL, sig, (mqtt.connected ? MQTT_SYMBOL : ' '), sensor.temperature, sensor.humidity, (int)sensor.altitude, (int)(sensor.pressure / 100));
	    #endif
            #if CONFIG_USE_BMP280
            snprintf(s_buffer, 100, "      %c%c      %c\n \nTemp:    %+3.1fC\n \nPressure:%4dhP", ANTENNA_SYMBOL, sig, (mqtt.connected ? MQTT_SYMBOL : ' '), sensor.temperature, (int)(sensor.pressure / 100));
	    #endif
            sh1106_display_text(s_buffer);
#endif
            if (wifi.connected){
                write_mqtt_data();
            }
            xSemaphoreGive(ctrl_sem1);
        }
    }
}

/* Read sensor data*/
static void task_read_sensors(void *arg)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(8000));
        if (xSemaphoreTake(ctrl_sem1, pdMS_TO_TICKS(10000)) == pdTRUE)
        {
#if (CONFIG_USE_DHT11)
            int _rVal = DHT11_read().humidity;
            if ((_rVal > MIN_HUM) && (_rVal < MAX_HUM))
            {
                sensor.humidity = (uint32_t)_rVal;
            }
            else
            {
                ESP_LOGE(TAG, "Error reading DHT11 humidity");
            }
#endif
#if (!CONFIG_USE_BMP180 && CONFIG_USE_DHT11)
            _rVal = DHT11_read().temperature;
            if ((_rVal > MIN_TEMP) && (_rVal < MAX_TEMP))
            {
                sensor.temperature = (float)_rVal;
            }
            else
            {
                ESP_LOGE(TAG, "Error reading DHT11 altitude");
            }
#endif
#if (CONFIG_USE_BMP180)
            u_int32_t bmp_pressure;
            float bmp_altitude;
            float bmp_temperature;
            esp_err_t err = bmp180_read_pressure(&bmp_pressure);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading BMP180, err = %d", err);
            }
            else
            {
                if ((bmp_pressure > MIN_PRESSURE) && (bmp_pressure < MAX_PRESSURE))
                {
                    sensor.pressure = bmp_pressure;
                }
                else
                {
                    ESP_LOGE(TAG, "Error reading BMP180, pressure out of range");
                }
            }
            err = bmp180_read_altitude(REFERENCE_PRESSURE, &bmp_altitude);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading BMP180, err = %d", err);
            }
            else
            {
                if ((bmp_altitude > MIN_ALTITUDE) && (bmp_altitude < MAX_ALTITUDE))
                {
                    sensor.altitude = bmp_altitude;
                }
                else
                {
                    ESP_LOGE(TAG, "Error reading BMP180, altitude out of range");
                }
            }
            err = bmp180_read_temperature(&bmp_temperature);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading BMP180, err = %d", err);
            }
            else
            {
                if ((bmp_temperature > MIN_TEMP) && bmp_temperature < MAX_TEMP)
                {
                    sensor.temperature = bmp_temperature;
                }
                else
                {
                    ESP_LOGE(TAG, "Error reading BMP180, temperature out of range");
                }
            }
#endif
#ifdef APP_DEBUG
            int st_size = STACK_SIZE - uxTaskGetStackHighWaterMark(xHandle);
            printf("Stack size used for data write task: %d \n", st_size);
            st_size = STACK_SIZE - uxTaskGetStackHighWaterMark(xHandle2);
            printf("Stacks size used for sensor read: %d \n", st_size);
            fflush(stdout);
#endif
            xSemaphoreGive(ctrl_sem1);
        }
    }
}

void app_main()
{
    //Initialize NVS necessary for Wifi functionality
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(i2c_master_init(I2C_PIN_SDA, I2C_PIN_SCL));


#if (CONFIG_USE_DHT11)
    //Initialize Humidity/Temperature sensor
    DHT11_init(GPIO_NUM_33);
#endif
#if (CONFIG_USE_BMP180)
    //Init BMP180 Sensor
    ESP_ERROR_CHECK(bmp180_init());
#endif
#if (CONFIG_USE_BMP280)
    while (i2cdev_init() != ESP_OK)
    {
        printf("Could not init I2Cdev library\n");
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    init_bme280();
#endif
#if (CONFIG_USE_SH1106)
    //Init OLED Display
    sh1106_init();
    sh1106_display_clear(NULL);
    //Write initial display message
    {
        char s_buffer[128];
	//snprintf(s_buffer, 100, "\nThe Tiny\n \nWeather\n \nStation\n \n(c)2021 W.Klum");
	snprintf(s_buffer, 100, "\n Weather\n \n Station\n");
        sh1106_display_text(s_buffer);
    }
#endif
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
#if (CONFIG_USE_MQTT)
    mqtt_app_start();
#endif
    ctrl_sem1 = xSemaphoreCreateBinary();
#if (CONFIG_USE_BMP180)
    xTaskCreatePinnedToCore(task_read_sensors, "readSensors", STACK_SIZE, (void *)1, uxTaskPriorityGet(NULL), NULL, 1);
#endif
#if (CONFIG_USE_BMP280)
    xTaskCreatePinnedToCore(bmp280_read, "bmp280_read", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
#endif
    xTaskCreatePinnedToCore(task_write_data, "writeDataTask", STACK_SIZE, (void *)1, uxTaskPriorityGet(NULL), NULL, 1);
    xSemaphoreGive(ctrl_sem1);

    initialise_weather_data_retrieval(WEATHER_DATA_RETREIVAL_PERIOD);
    on_weather_data_retrieval(weather_data_retreived);
    ESP_LOGW(TAG, "Weather data retrieval initialized");
}
