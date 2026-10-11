/* LoRa gönder + dinle denemesi (03-lora-txrx)

   Her kart aynı kodu çalıştırır:
   - Radyo zamanının çoğunda dinler, gelen paketi ekrana ve loga yazar.
   - 5-6 saniyede bir kendi paketini yayınlar, sonra hemen dinlemeye döner.

   LED kodu bu denemede yok; LED + ekran hali experiments/01-blink-oled içinde duruyor.
*/

// ---------- Kütüphaneler ----------
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "sdkconfig.h"
#include "ssd1306.h"

static const char *TAG = "lora_txrx";

// ---------- Pinler ve SX1278 yazmaçları ----------

// T3 V1.6.1: ESP32 ile SX1278 arasındaki pinler
#define LORA_PIN_SCK   5
#define LORA_PIN_MISO  19
#define LORA_PIN_MOSI  27
#define LORA_PIN_CS    18
#define LORA_PIN_RST   23

// SX1278 yazmaç adresleri (isimler veri sayfasındakiyle aynı)
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10   // son gelen paketin FIFO'daki başlangıcı
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13   // son gelen paketin uzunluğu
#define REG_PKT_SNR_VALUE        0x19   // son paketin SNR değeri
#define REG_PKT_RSSI_VALUE       0x1A   // son paketin sinyal gücü
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_VERSION              0x42

// RegOpMode değerleri: en üst bit 1 = LoRa modu
#define MODE_SLEEP          0x80
#define MODE_STANDBY        0x81
#define MODE_TX             0x83
#define MODE_RX_CONTINUOUS  0x85        // sürekli dinle

// RegIrqFlags bitleri
#define IRQ_TX_DONE            0x08     // gönderim bitti
#define IRQ_PAYLOAD_CRC_ERROR  0x20     // paket bozuk geldi
#define IRQ_RX_DONE            0x40     // paket geldi

#define SX1278_VERSION   0x12           // RegVersion'ın beklenen değeri
#define RSSI_OFFSET_LF   (-164)         // 433 MHz (düşük frekans portu) için veri sayfasındaki sabit

static spi_device_handle_t s_lora_spi;

// ---------- Ekran (senin kodun) ----------

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

// Bu denemede kullanılmıyor, ileride mesaj göstermek için duruyor
static void draw_text_read(const char *text, const char *esp_id)
{
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

// Ekran: üstte kimlik, altında son gönderim, son alım ve sinyal bilgisi
static void display_status(const char *device_name, const char *tx_line,
                           const char *rx_line, const char *signal_line)
{
    ssd1306_clear(ekran);
    ssd1306_draw_text(ekran, 0, 0,  device_name, true);
    ssd1306_draw_text(ekran, 0, 16, tx_line,     true);
    ssd1306_draw_text(ekran, 0, 30, rx_line,     true);
    ssd1306_draw_text(ekran, 0, 42, signal_line, true);
    ssd1306_display(ekran);
}

// ---------- Radyo: SPI ile yazmaç erişimi ----------

static void lora_write_reg(uint8_t reg, uint8_t value)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,                          // 2 bayt = 16 bit
        .tx_data = { reg | 0x80, value },      // en üst bit 1: yaz
    };
    spi_device_polling_transmit(s_lora_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = { reg & 0x7F, 0x00 },       // en üst bit 0: oku
    };
    spi_device_polling_transmit(s_lora_spi, &t);
    return t.rx_data[1];                       // cevap 2. baytta gelir
}

static void lora_write_fifo(const uint8_t *data, int length)
{
    static WORD_ALIGNED_ATTR uint8_t buffer[256];   // DMA için kalıcı ve hizalı
    buffer[0] = REG_FIFO | 0x80;
    memcpy(&buffer[1], data, length);

    spi_transaction_t t = {
        .length = (length + 1) * 8,
        .tx_buffer = buffer,
    };
    spi_device_polling_transmit(s_lora_spi, &t);
}

// ---------- Radyo: başlatma ----------

static void lora_init(void)
{
    // 1. SPI veriyolu
    spi_bus_config_t bus_config = {
        .mosi_io_num = LORA_PIN_MOSI,
        .miso_io_num = LORA_PIN_MISO,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    // 2. SX1278'i veriyoluna ekle
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,    // 1 MHz
        .mode = 0,
        .spics_io_num = LORA_PIN_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device_config, &s_lora_spi));

    // 3. Çipi sıfırla
    gpio_reset_pin(LORA_PIN_RST);
    gpio_set_direction(LORA_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 4. İletişim testi
    uint8_t chip_version = lora_read_reg(REG_VERSION);
    ESP_LOGI(TAG, "SX1278 version: 0x%02X", chip_version);
    if (chip_version != SX1278_VERSION) {
        ESP_LOGE(TAG, "Radio chip not responding! Check pins.");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    // 5. Radyo ayarları (üç kartta aynı olmalı)
    lora_write_reg(REG_OP_MODE, MODE_SLEEP);        // LoRa modu sadece uykuda açılır
    vTaskDelay(pdMS_TO_TICKS(10));

    lora_write_reg(REG_FRF_MSB, 0x6C);              // 433,5 MHz = 0x6C6000
    lora_write_reg(REG_FRF_MID, 0x60);
    lora_write_reg(REG_FRF_LSB, 0x00);

    lora_write_reg(REG_PA_CONFIG, 0x88);            // PA_BOOST, ~10 dBm
    lora_write_reg(REG_MODEM_CONFIG_1, 0x72);       // BW 125 kHz, CR 4/5, açık başlık
    lora_write_reg(REG_MODEM_CONFIG_2, 0x94);       // SF9, CRC açık
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);       // otomatik kazanç

    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);

    lora_write_reg(REG_OP_MODE, MODE_STANDBY);
    ESP_LOGI(TAG, "Radio ready: 433.5 MHz, SF9, BW125");
}

// ---------- Radyo: gönderme ----------

static void lora_send(const uint8_t *data, int length)
{
    lora_write_reg(REG_OP_MODE, MODE_STANDBY);      // dinlemeyi bırak
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_fifo(data, length);
    lora_write_reg(REG_PAYLOAD_LENGTH, length);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);            // eski bayrakları temizle
    lora_write_reg(REG_OP_MODE, MODE_TX);

    while ((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
}

// ---------- Radyo: dinleme ----------

static void lora_start_receive(void)
{
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS);
}

// Paket geldiyse okur.
// Dönüş: paket uzunluğu, paket yoksa 0, bozuk paket geldiyse -1
static int lora_receive(uint8_t *data, int max_length, int *rssi, float *snr)
{
    uint8_t flags = lora_read_reg(REG_IRQ_FLAGS);
    if ((flags & IRQ_RX_DONE) == 0) {
        return 0;                                   // henüz paket yok
    }
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);            // bayrakları temizle

    if (flags & IRQ_PAYLOAD_CRC_ERROR) {
        return -1;                                  // paket havada bozulmuş
    }

    int length = lora_read_reg(REG_RX_NB_BYTES);
    if (length > max_length) {
        length = max_length;                        // tampondan taşma olmasın
    }

    // FIFO'da paketin başladığı yere git, baytları tek tek oku
    lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    for (int i = 0; i < length; i++) {
        data[i] = lora_read_reg(REG_FIFO);          // her okumada FIFO işaretçisi 1 ilerler
    }

    *rssi = RSSI_OFFSET_LF + lora_read_reg(REG_PKT_RSSI_VALUE);
    *snr = (int8_t)lora_read_reg(REG_PKT_SNR_VALUE) / 4.0f;
    return length;
}

// ---------- Paket yapısı ----------

typedef struct __attribute__((packed)) {
    uint16_t sender_id;    // gönderenin kimliği (MAC'in son 2 baytı)
    uint16_t sequence;     // her pakette 1 artar
    char text[20];
} test_packet_t;           // toplam 24 bayt

// ---------- Ana program ----------

void app_main(void)
{
    ekran_baslat();

    char esp_id[16];
    read_mac_id(esp_id, sizeof(esp_id));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint16_t my_id = (mac[4] << 8) | mac[5];

    lora_init();
    lora_start_receive();

    // Gönderim durumu: ilk gönderim 1-3 sn sonra (iki kart aynı anda açılırsa çarpışmasın)
    uint16_t tx_sequence = 0;
    int64_t next_tx_time_us = esp_timer_get_time() + (1000 + esp_random() % 2000) * 1000LL;

    // Alım istatistikleri
    uint16_t last_rx_sequence = 0;
    bool has_received = false;
    uint32_t received_count = 0;
    uint32_t lost_count = 0;

    // Ekran satırları
    char tx_line[40] = "TX -";
    char rx_line[40] = "RX bekleniyor...";
    char signal_line[40] = "";
    display_status(esp_id, tx_line, rx_line, signal_line);

    uint8_t rx_buffer[255];

    while (1) {
        // ---- 1. Paket geldi mi? ----
        int rssi = 0;
        float snr = 0;
        int length = lora_receive(rx_buffer, sizeof(rx_buffer), &rssi, &snr);

        if (length == sizeof(test_packet_t)) {
            test_packet_t packet;
            memcpy(&packet, rx_buffer, sizeof(packet));     // baytları struct'a geri çevir

            // Kayıp paket hesabı: sıra numarasında atlama var mı?
            if (has_received && packet.sequence != (uint16_t)(last_rx_sequence + 1)) {
                lost_count += (uint16_t)(packet.sequence - last_rx_sequence - 1);
            }
            last_rx_sequence = packet.sequence;
            has_received = true;
            received_count++;

            ESP_LOGI(TAG, "RX from ESP-%04X #%u \"%s\" rssi=%d snr=%.1f received=%lu lost=%lu",
                     packet.sender_id, packet.sequence, packet.text, rssi, snr,
                     received_count, lost_count);

            snprintf(rx_line, sizeof(rx_line), "RX %04X #%u", packet.sender_id, packet.sequence);
            snprintf(signal_line, sizeof(signal_line), "%ddBm ok:%lu x:%lu", rssi, received_count, lost_count);
            display_status(esp_id, tx_line, rx_line, signal_line);
        } else if (length == -1) {
            ESP_LOGW(TAG, "RX CRC error (corrupted packet)");
        } else if (length > 0) {
            ESP_LOGW(TAG, "RX unexpected length: %d bytes", length);
        }

        // ---- 2. Gönderme zamanı geldi mi? ----
        if (esp_timer_get_time() >= next_tx_time_us) {
            test_packet_t packet = {0};
            packet.sender_id = my_id;
            packet.sequence = tx_sequence;
            snprintf(packet.text, sizeof(packet.text), "merhaba");

            int64_t start_us = esp_timer_get_time();
            lora_send((uint8_t *)&packet, sizeof(packet));
            int64_t airtime_ms = (esp_timer_get_time() - start_us) / 1000;
            lora_start_receive();                           // hemen dinlemeye dön

            ESP_LOGI(TAG, "TX #%u sent, %lld ms", tx_sequence, airtime_ms);
            snprintf(tx_line, sizeof(tx_line), "TX #%u %lldms", tx_sequence, airtime_ms);
            display_status(esp_id, tx_line, rx_line, signal_line);

            tx_sequence++;
            next_tx_time_us = esp_timer_get_time() + (5000 + esp_random() % 1000) * 1000LL;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}