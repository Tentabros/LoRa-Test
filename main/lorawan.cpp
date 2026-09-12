#include <RadioLib.h>
#include "EspHal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "sender";

//-------------------------Radio Pins---------------------//
// Double-check against your board (e.g. XIAO B2B pins: SCK=7, MISO=8, MOSI=9, NSS=41/5)
#define LORA_SCK  30   
#define LORA_MISO 31
#define LORA_MOSI 37
#define nss       29
#define dio_1     1
#define nrst      7
#define busy      6

// Optional external RF switch pin on some Wio carrier boards (Set to -1 if unused)
#define ANT_SW_PIN GPIO_NUM_38 

//-------------------------Radio Config---------------------//
#define LORA_FREQ     915.0
#define LORA_BW       125.0
#define LORA_SF       7
#define LORA_CR       5
#define LORA_TX_POWER 22
#define TX_INTERVAL_MS 1000

//-------------------------Packet Layout---------------------//
struct __attribute__((packed)) TelemetryPacket
{
   uint8_t  seq;
   int32_t  lat_e7;
   int32_t  lon_e7;
   int16_t  speed_x10;
   int16_t  mc_temp_x10;
   int16_t  batt_temp_x10;
   int16_t  voltage_x10;
   int16_t  current1_x10;
   int16_t  current2_x10;
   int16_t  power_x10;
   uint8_t  throttle_pct;
   int16_t  efficiency_x100;
   int16_t  energy;
   int16_t  kill_flag_val;
};

TelemetryPacket packet;
uint8_t seqCounter = 0;

EspHal *hal = new EspHal(LORA_SCK, LORA_MISO, LORA_MOSI);
SX1262 radio = new Module(hal, nss, dio_1, nrst, busy);

extern "C" void app_main(void)
{
   ESP_LOGI(TAG, "Initializing hardware...");

   // Power on external RF Switch pin if your carrier board requires it
   if (ANT_SW_PIN >= 0) {
       gpio_reset_pin(ANT_SW_PIN);
       gpio_set_direction(ANT_SW_PIN, GPIO_MODE_OUTPUT);
       gpio_set_level(ANT_SW_PIN, 1);
   }

   // radio.begin parameter order:
   // freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO
   int state = radio.begin(
       LORA_FREQ, 
       LORA_BW, 
       LORA_SF, 
       LORA_CR, 
       RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 
       LORA_TX_POWER, 
       8,    // Preamble length
       1.6   // TCXO voltage (1.6V required for Wio-SX1262)
   );

   if (state != RADIOLIB_ERR_NONE) {
       ESP_LOGE(TAG, "Radio init failed, code %d", state);
       while (true) { 
           vTaskDelay(pdMS_TO_TICKS(1000)); 
       }
   }

   radio.setDio2AsRfSwitch(true);
   ESP_LOGI(TAG, "Radio initialized successfully!");

   int64_t lastTxTime = esp_timer_get_time() / 1000;

   for (;;) {
       int64_t now = esp_timer_get_time() / 1000;
       if (now - lastTxTime >= TX_INTERVAL_MS) {
           lastTxTime = now;

           packet.seq             = seqCounter++;
           packet.lat_e7          = 145646000 + seqCounter;
           packet.lon_e7          = 1209932000 + seqCounter;
           packet.speed_x10       = 100 + seqCounter;
           packet.mc_temp_x10     = 500 + seqCounter;
           packet.batt_temp_x10   = 400 + seqCounter;
           packet.voltage_x10     = 4800 + seqCounter;
           packet.current1_x10    = 150 + seqCounter;
           packet.current2_x10    = 120 + seqCounter;
           packet.power_x10       = 7200 + seqCounter;
           packet.throttle_pct    = seqCounter % 100;
           packet.efficiency_x100 = 1800 + seqCounter;
           packet.energy          = 500 + seqCounter;
           packet.kill_flag_val   = 0;

           int txState = radio.transmit((uint8_t*)&packet, sizeof(packet));
           if (txState == RADIOLIB_ERR_NONE) {
               ESP_LOGI(TAG, "Sent seq #%d | Speed_x10=%d | Voltage_x10=%d", 
                        packet.seq, packet.speed_x10, packet.voltage_x10);
           } else {
               ESP_LOGE(TAG, "TX failed, code %d", txState);
           }
       }
       vTaskDelay(pdMS_TO_TICKS(10));
   }
}