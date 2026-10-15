/* LoRa Mesh – ilk mesh denemesi (04-mesh-relay)

   Üç karta da AYNI kod yüklenir. Her kart açılışta kendi kimliğini MAC adresinden okur
   ve aşağıdaki SENDER_ID / RECEIVER_ID değerlerine bakarak rolünü anlar:

     - Kimliği SENDER_ID olan kart: açılıştan birkaç saniye sonra RECEIVER_ID'ye bir mesaj gönderir.
     - Kimliği RECEIVER_ID olan kart: mesajı ekrana yazar ve göndericiye ONAY yollar.
     - Diğer kartlar: aracı. Kendilerine ait olmayan paketleri rotaya kimliklerini ekleyip tekrar yayar.

   Mesajı tekrar göndermek için gönderen kartın reset düğmesine bas.

   Karar sırası (her kart, her paket için):
     1. (kaynak, mesaj no, tip) daha önce görüldü mü?  -> evet: at
     2. Hedef ben miyim?
          tip 2 (ONAY)  -> "Mesaj gitti"
          tip 1 (MESAJ) -> mesajı göster, ONAY gönder
     3. Aracıyım: rotada varsam ya da rota doluysa at; değilsem rotaya eklen ve yay
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

static const char *TAG = "mesh";

// =====================================================================
// AYARLAR: Kartlarının kimliklerini buraya yaz
// (Her kart açılışta ekranın sol üstünde kimliğini gösterir: ESP-XXXX)
// =====================================================================

#define SENDER_ID      0x5A00    // mesajı gönderecek kartın kimliği   <-- KENDİ DEĞERİNİ YAZ
#define RECEIVER_ID    0x82C0     // mesajı alacak kartın kimliği       <-- KENDİ DEĞERİNİ YAZ

#define MESSAGE_TEXT   "Telefonum kapandi"

// Masa testi kuralı: alıcı, gönderenden DOĞRUDAN gelen paketleri yok sayar.
// Böylece kartlar yan yanayken bile mesaj ancak aracı kart üzerinden ulaşır.
// Sahada (kartlar uzaktayken) 0 yap.
#define TEST_IGNORE_DIRECT  1

#define ACK_TIMEOUT_MS      15000  // gönderen bu süre içinde onay alamazsa ekrana yazar

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

// ---------- Mesh paket yapısı ----------

#define MSG_TYPE_MESSAGE  1
#define MSG_TYPE_ACK      2
#define MAX_HOPS          5          // rotaya en fazla 5 kimlik sığar

typedef struct __attribute__((packed)) {
    uint8_t  type;                   // 1 = mesaj, 2 = onay
    uint16_t msg_id;                 // gönderenin mesaj numarası (onayda aynı kalır)
    uint16_t source_id;              // paketi oluşturan kart
    uint16_t dest_id;                // hedef kart
    uint8_t  hop_count;              // rotada kaç kimlik var
    uint16_t route[MAX_HOPS];        // paketin geçtiği kartlar
    char     text[48];               // mesaj içeriği
} mesh_packet_t;                     // 1+2+2+2+1+10+48 = 66bayt

// ---------- Bu kartın durumu ----------

static uint16_t s_my_id;             // MAC'ten okunan kimlik (ör. 0x3A7C)
static char     s_my_name[16];       // ekranda gösterilen kimlik (ör. "ESP-3A7C")
static uint16_t s_next_msg_id;       // bir sonraki mesajın numarası (açılışta rastgele başlar)

// Gönderen kart için: onay bekleniyor mu?
static bool     s_waiting_ack = false;
static uint16_t s_waiting_msg_id = 0;
static int64_t  s_ack_deadline_us = 0;

// =====================================================================
// EKRAN
// =====================================================================

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

static void read_mac_id(char *ad, int boyut)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(ad, boyut, "ESP-%02X%02X", mac[4], mac[5]);
}

// Ekran düzeni: en üstte kartın kimliği, altında 4 satır bilgi.
// Boş bırakılacak satır için "" ver.
static void display_show(const char *line1, const char *line2,
                         const char *line3, const char *line4)
{
    ssd1306_clear(ekran);
    ssd1306_draw_text(ekran, 0, 0,  s_my_name, true);
    ssd1306_draw_text(ekran, 0, 16, line1, true);
    ssd1306_draw_text(ekran, 0, 28, line2, true);
    ssd1306_draw_text(ekran, 0, 40, line3, true);
    ssd1306_draw_text(ekran, 0, 52, line4, true);   // 52 + 8 = 60: ekrana sığar
    ssd1306_display(ekran);
}

// Uzun bir metnin 'start'. karakterinden itibaren en fazla 21 karakterini 'out'a kopyalar
// (ekranın bir satırına 21 karakter sığıyor)
static void text_slice(const char *text, int start, char *out, int out_size)
{
    int len = strlen(text);
    if (start >= len) {
        out[0] = '\0';
        return;
    }
    snprintf(out, out_size, "%.21s", text + start);
}

// =====================================================================
// RADYO: SPI ile yazmaç erişimi
// =====================================================================

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

// =====================================================================
// RADYO: başlatma, gönderme, dinleme
// =====================================================================

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
        display_show("RADYO HATASI", "SX1278 cevap", "vermiyor", "");
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

static void lora_send(const uint8_t *data, int length)
{
    lora_write_reg(REG_OP_MODE, MODE_STANDBY);      // dinlemeyi bırak
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);        // FIFO başına git
    lora_write_fifo(data, length);                  // veriyi FIFO'ya yaz
    lora_write_reg(REG_PAYLOAD_LENGTH, length);     // gönderilecek uzunluğu yaz
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);            // eski bayrakları temizle
    lora_write_reg(REG_OP_MODE, MODE_TX);           // gönderime başla

    while ((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0) {   // gönderim bitene kadar bekle
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);            // bayrakları temizle
}

static void lora_start_receive(void)
{
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);        // FIFO başına git
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);            // eski bayrakları temizle
    lora_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS); // sürekli dinlemeye başla
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

    int length = lora_read_reg(REG_RX_NB_BYTES);    // kaç bayt geldi?
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

// =====================================================================
// MESH: yardımcı fonksiyonlar
// =====================================================================

// ---- "Gördüklerim" listesi: son 16 paketin (kaynak, mesaj no, tip) üçlüsü ----

typedef struct {
    uint16_t source_id;
    uint16_t msg_id;
    uint8_t  type;
} seen_entry_t;

#define SEEN_SIZE 16
static seen_entry_t s_seen[SEEN_SIZE];
static int s_seen_next = 0;          // bir sonraki kaydın yazılacağı yer
static int s_seen_count = 0;         // listede kaç kayıt var

// Daha önce gördüysem true döner. Görmediysem listeye ekler ve false döner.
static bool seen_before(const mesh_packet_t *p)
{
    for (int i = 0; i < s_seen_count; i++) {
        if (s_seen[i].source_id == p->source_id &&
            s_seen[i].msg_id    == p->msg_id &&
            s_seen[i].type      == p->type) {
            return true;
        }
    }
    s_seen[s_seen_next].source_id = p->source_id;
    s_seen[s_seen_next].msg_id    = p->msg_id;
    s_seen[s_seen_next].type      = p->type;
    s_seen_next = (s_seen_next + 1) % SEEN_SIZE;          // 15'ten sonra 0'a dön
    if (s_seen_count < SEEN_SIZE) {
        s_seen_count++;
    }
    return false;
}

// Bu kimlik paketin rotasında var mı?
static bool route_contains(const mesh_packet_t *p, uint16_t id)
{
    for (int i = 0; i < p->hop_count && i < MAX_HOPS; i++) {
        if (p->route[i] == id) {
            return true;
        }
    }
    return false;
}

// Rotayı "3A7C>82C0>91F0" gibi bir metne çevirir
static void route_to_str(const mesh_packet_t *p, char *out, int out_size)
{
    int pos = 0;
    out[0] = '\0';
    for (int i = 0; i < p->hop_count && i < MAX_HOPS; i++) {
        int written = snprintf(out + pos, out_size - pos, "%s%04X", (i > 0) ? ">" : "", p->route[i]);
        if (written < 0 || written >= out_size - pos) {
            break;                                  // yer kalmadı
        }
        pos += written;
    }
}

static const char *type_name(uint8_t type)
{
    return (type == MSG_TYPE_ACK) ? "ONAY" : "MESAJ";
}

// Paketi gönderir: önce kendi listeme ekle (geri dönerse tanıyayım), sonra yay, sonra dinlemeye dön
static void mesh_send(mesh_packet_t *p)
{
    seen_before(p);                                 // sonucu önemli değil, amaç listeye eklemek

    char route_str[32];
    route_to_str(p, route_str, sizeof(route_str));

    int64_t start_us = esp_timer_get_time();
    lora_send((uint8_t *)p, sizeof(*p));
    int64_t airtime_ms = (esp_timer_get_time() - start_us) / 1000;
    lora_start_receive();

    ESP_LOGI(TAG, "TX %s #%04X %04X->%04X rota=%s (%lld ms)",
             type_name(p->type), p->msg_id, p->source_id, p->dest_id, route_str, airtime_ms);
}

// =====================================================================
// MESH: paket geldiğinde verilen karar
// =====================================================================

static void handle_packet(mesh_packet_t *p, int rssi)
{
    char route_str[32];
    route_to_str(p, route_str, sizeof(route_str));

    ESP_LOGI(TAG, "RX %s #%04X %04X->%04X rota=%s rssi=%d",
             type_name(p->type), p->msg_id, p->source_id, p->dest_id, route_str, rssi);

#if TEST_IGNORE_DIRECT
    // Masa testi: alıcı, gönderenden doğrudan gelen MESAJI yok sayar (aktarma görülebilsin diye)
    if (s_my_id == RECEIVER_ID && p->type == MSG_TYPE_MESSAGE && p->hop_count == 1) {
        ESP_LOGW(TAG, "  -> test kurali: dogrudan gelen mesaj yok sayildi");
        return;
    }
    if (s_my_id == SENDER_ID && p->type == MSG_TYPE_ACK && p->hop_count == 1) {
        ESP_LOGW(TAG, "  -> test kurali: dogrudan gelen onay yok sayildi");
        return;
    }
#endif

    // ---- 1. Kopya kontrolü ----
    if (seen_before(p)) {
        ESP_LOGI(TAG, "  -> daha once goruldu, atildi: #%04X %04X->%04X", p->msg_id, p->source_id, p->dest_id);
        return;
    }

    char line1[40], line2[40], line3[40], line4[40];

    // ---- 2. Hedef ben miyim? ----
    if (p->dest_id == s_my_id) {

        if (p->type == MSG_TYPE_ACK) {
            // Gönderdiğim mesajın onayı geldi
            ESP_LOGI(TAG, "  -> ONAY bana geldi: mesaj #%04X ulasti", p->msg_id);
            if (s_waiting_ack && p->msg_id == s_waiting_msg_id) {
                s_waiting_ack = false;
            }
            snprintf(line1, sizeof(line1), "MESAJ GITTI");
            snprintf(line2, sizeof(line2), "#%04X -> %04X", p->msg_id, p->source_id);
            snprintf(line3, sizeof(line3), "onay rotasi:");
            snprintf(line4, sizeof(line4), "%s", route_str); // onayın rotası
            display_show(line1, line2, line3, line4);
            return;
        }

        // MESAJ bana geldi: göster
        ESP_LOGI(TAG, "  -> MESAJ bana geldi: \"%s\"", p->text);
        snprintf(line1, sizeof(line1), "Kimden: ESP-%04X", p->source_id);
        text_slice(p->text, 0,  line2, sizeof(line2));
        text_slice(p->text, 21, line3, sizeof(line3));
        snprintf(line4, sizeof(line4), "%s", route_str);
        display_show(line1, line2, line3, line4);

        // ONAY hazırla: aynı paket, yönü ters
        mesh_packet_t ack = *p;                     // metin ve msg_id aynı kalır
        ack.type      = MSG_TYPE_ACK;
        ack.dest_id   = p->source_id;               // göndericiye
        ack.source_id = s_my_id;                    // kendimden
        for (int i = 0; i < MAX_HOPS; i++) {        // rotayı sıfırla
            ack.route[i] = 0;
        }
        ack.route[0]  = s_my_id;                    // sadece kendimi yaz
        ack.hop_count = 1;

        vTaskDelay(pdMS_TO_TICKS(100 + esp_random() % 400));   // aracı aktarırken çarpışmasın
        mesh_send(&ack);
        return;
    }

    // ---- 3. Aracıyım: tipe bakmadan aktar ----
    if (route_contains(p, s_my_id)) {
        ESP_LOGI(TAG, "  -> rotada zaten varim, yaymiyorum");
        return;
    }
    if (p->hop_count >= MAX_HOPS) {
        ESP_LOGW(TAG, "  -> rota dolu (%d), paket olur", p->hop_count);
        return;
    }

    snprintf(line1, sizeof(line1), "Paket var, alici");
    snprintf(line2, sizeof(line2), "ben degilim (%s)", type_name(p->type));
    snprintf(line3, sizeof(line3), "%04X>%04X #%04X", p->source_id, p->dest_id, p->msg_id);
    snprintf(line4, sizeof(line4), "%ddBm aktariliyor", rssi);
    display_show(line1, line2, line3, line4);

    mesh_packet_t relay = *p;
    relay.route[relay.hop_count] = s_my_id;         // kendimi rotaya ekle
    relay.hop_count++;
    char route_str2[32];
    route_to_str(  &relay, route_str2, sizeof(route_str2));
    ESP_LOGI(TAG, "  -> aracilik: rota=%s", route_str2);
    vTaskDelay(pdMS_TO_TICKS(100 + esp_random() % 400));       // birden fazla aracı varsa çarpışmasın
    mesh_send(&relay);
}

// =====================================================================
// ANA PROGRAM
// =====================================================================

void app_main(void)
{
    ekran_baslat();

    // Kimliğimi MAC adresinden oku
    read_mac_id(s_my_name, sizeof(s_my_name));
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    s_my_id = (mac[4] << 8) | mac[5];
    ESP_LOGW(TAG, "  -> s_my_id: %04X", s_my_id);
    // Mesaj numarası rastgele başlasın: kart reset'lenince eski numaralarla karışmasın
    s_next_msg_id = esp_random() & 0xFFFF;

    // Rolümü belirle
    const char *role;
    bool has_message_to_send = false;           // ileride bu değişkeni telefon açacak
    if (s_my_id == SENDER_ID) {
        role = "GONDEREN";
        has_message_to_send = true;
    } else if (s_my_id == RECEIVER_ID) {
        role = "ALICI";
    } else {
        role = "ARACI";
    }
    ESP_LOGI(TAG, "Ben %s (%04X), rol: %s", s_my_name, s_my_id, role);

    char line1[40];
    snprintf(line1, sizeof(line1), "Rol: %s", role);
    display_show(line1, "Dinleniyor...", "", "");

    lora_init();
    lora_start_receive();

    // Gönderen kart mesajı açılıştan 3 saniye sonra yollar
    int64_t send_time_us = esp_timer_get_time() + 3000000LL;

    uint8_t rx_buffer[255];

    while (1) {
        // ---- 1. Paket geldi mi? ----
        int rssi = 0;
        float snr = 0;
        int length = lora_receive(rx_buffer, sizeof(rx_buffer), &rssi, &snr);

        if (length == sizeof(mesh_packet_t)) {
            mesh_packet_t p;
            memcpy(&p, rx_buffer, sizeof(p));       // baytları struct'a çevir
            handle_packet(&p, rssi);
        } else if (length == -1) {
            ESP_LOGW(TAG, "RX CRC hatasi (bozuk paket)");
        } else if (length > 0) {
            ESP_LOGW(TAG, "RX beklenmeyen uzunluk: %d bayt", length);
        }

        // ---- 2. Gönderilecek mesaj var mı? ----
        if (has_message_to_send && esp_timer_get_time() >= send_time_us) {
            has_message_to_send = false;

            mesh_packet_t msg = {0};                // her şeyi sıfırla
            msg.type      = MSG_TYPE_MESSAGE;
            msg.msg_id    = s_next_msg_id++;
            msg.source_id = s_my_id;
            msg.dest_id   = RECEIVER_ID;
            msg.route[0]  = s_my_id;                // rotanın ilk elemanı benim
            msg.hop_count = 1;
            snprintf(msg.text, sizeof(msg.text), "%s", MESSAGE_TEXT);

            mesh_send(&msg);

            s_waiting_ack     = true;
            s_waiting_msg_id  = msg.msg_id;
            s_ack_deadline_us = esp_timer_get_time() + ACK_TIMEOUT_MS * 1000LL;

            char l1[40], l2[40];
            snprintf(l1, sizeof(l1), "Gonderildi #%04X", msg.msg_id);
            snprintf(l2, sizeof(l2), "-> ESP-%04X", RECEIVER_ID);
            display_show(l1, l2, "Onay bekleniyor...", "");
        }

        // ---- 3. Onay zamanında gelmedi mi? ----
        if (s_waiting_ack && esp_timer_get_time() >= s_ack_deadline_us) {
            s_waiting_ack = false;
            ESP_LOGW(TAG, "Mesaj #%04X icin onay gelmedi", s_waiting_msg_id);
            char l2[40];
            snprintf(l2, sizeof(l2), "#%04X", s_waiting_msg_id);
            display_show("ONAY GELMEDI", l2, "Tekrar icin", "reset'e bas");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}