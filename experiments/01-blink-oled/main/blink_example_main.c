/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <sys/_intsup.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "esp_mac.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

static ssd1306_handle_t ekran;

static void ekran_baslat(void)
{
    i2c_master_bus_config_t bus_ayar = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 21,             // T3'te ekranın SDA pini
        .scl_io_num = 22,             // T3'te ekranın SCL pini
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    i2c_new_master_bus(&bus_ayar, &bus);

    ssd1306_config_t ekran_ayar = {
        .bus = SSD1306_I2C,
        .width = 128,
        .height = 64,
        .iface.i2c = {
            .port = I2C_NUM_0,
            .addr = 0x3C,             // ekranın I2C adresi
            .rst_gpio = -1,           // T3'te ekranın ayrı reset pini yok
        },
    };
    ssd1306_new_i2c(&ekran_ayar, &ekran);
}

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void draw_text_read(const char *text,const char *esp_id){
    ssd1306_draw_text(ekran, 0, 0, esp_id, true);
    int uzunluk = strlen(text);

    int satir_sayisi = (uzunluk + 20) / 21;     // bir satıra 21 harf sığar
    if (satir_sayisi > 6) {
        satir_sayisi = 6;                       // ekrana en fazla 6 satır sığar
    }

    int y = (64 - satir_sayisi * 10) / 2;       // 64: ekran yüksekliği, 10: satır yüksekliği

    for (int i = 0; i < satir_sayisi; i++) {
        const char *bas = text + i * 21;
        int harf = strlen(bas);
        if (harf > 21) {
            harf = 21;
        }

        char satir[22];                         // 21 harf + '\0'
        memcpy(satir, bas, harf);
        satir[harf] = '\0';

        int x = (128 - harf * 6) / 2;           // 128: ekran genişliği, 6: harf genişliği
        
        ssd1306_draw_text(ekran, x, y + i * 10, satir, true);
    }
}
static void read_mac_id(char *ad, int boyut)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(ad, boyut, "ESP-%02X%02X", mac[4], mac[5]);
}
#else
#error "unsupported LED type"
#endif

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    ekran_baslat();
    
    char esp_id[16];
    read_mac_id(esp_id, sizeof(esp_id));

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        ssd1306_clear(ekran);

        draw_text_read(s_led_state ? "Led acik" : "Led kapali",esp_id);
        ssd1306_display(ekran);

        s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
