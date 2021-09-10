#include <Arduino.h>
#include <pins.h>
#include <sensors.h>
#include <canbus/kwp_server.h>

#include <esp_freertos_hooks.h>

#include <pwm_channels/channels.h>


static uint64_t ticks_core0 = 0;
static uint64_t ticks_core1 = 0;

int spkr_channel = 0;

#define TICKS_IDLE_PER_SECOND 1085215 // At 240Mhz

static bool idle_core0() {
  ticks_core0+=1;
  return false;
}

static bool idle_core1() {
  ticks_core1+=1;
  return false;
}


uint8_t duty = 10;

void y3_test(void* pvParams) {
  while(true) {
    tcc_pwm.write_pwm(duty);
    vTaskDelay(10);
  }
}

[[noreturn]] void print_task(void* pvParams) {
  while(true) {
    char buf[128];

    float l0 = 100.f - ((float)min(ticks_core0, (uint64_t)TICKS_IDLE_PER_SECOND) / TICKS_IDLE_PER_SECOND) * 100.f;
    float l1 = 100.f - ((float)min(ticks_core1, (uint64_t)TICKS_IDLE_PER_SECOND) / TICKS_IDLE_PER_SECOND) * 100.f;

    sprintf(buf, "CORE 0: %.1f%% CORE 1: %.1f%%. Free heap: %lu, Free PSRAM: %lu", l0, l1, ESP.getFreeHeap(), ESP.getFreePsram());
    ticks_core0 = 0;
    ticks_core1 = 0;
    
    Serial.println(buf);
    Serial.print("Y3:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::Y3));
    Serial.print(" mA, ");
    Serial.print("Y4:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::Y4));
    Serial.print(" mA, ");
    Serial.print("Y5:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::Y5));
    Serial.print(" mA, ");
    Serial.print("MPC:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::MPC));
    Serial.print(" mA, ");
    Serial.print("SPC:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::SPC));
    Serial.print(" mA, ");
    Serial.print("TCC:");
    Serial.print(Sensors::read_solenoid_current(Solenoid::TCC));
    Serial.println(" mA");
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SPKR, OUTPUT);

  ledcSetup(spkr_channel, 2000, 8);
  ledcAttachPin(PIN_SPKR, spkr_channel);
  //ledcWriteTone(spkr_channel, 500);

  Sensors::configure_sensor_pins();
  esp_register_freertos_idle_hook_for_cpu(idle_core0, 0);
  esp_register_freertos_idle_hook_for_cpu(idle_core1, 1);
  xTaskCreate(&print_task, "printer", 4096, nullptr, 1, nullptr);
  xTaskCreate(&y3_test, "test", 512, nullptr, 1, nullptr);
}


// Don't add any code here. Use RTOSTasks only
// This loop just idles forever
void loop() { vTaskDelay(10000); }