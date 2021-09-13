#include <Arduino.h>
#include <pins.h>
#include <sensors.h>
#include <config.h>
#include <canbus/kwp_server.h>
#include <canbus/egs52_can.h>

#include <esp_freertos_hooks.h>

#include <pwm_channels/channels.h>


static volatile uint64_t ticks_core0 = 0;
static volatile uint64_t ticks_core1 = 0;

int spkr_channel = 0;


void send_note(uint8_t channel, uint32_t freq, uint32_t dur, uint32_t total_dur) {
  ledcWriteTone(spkr_channel, freq);
  vTaskDelay(dur);
  ledcWriteTone(spkr_channel, 0);
  vTaskDelay(total_dur - dur);
}

[[noreturn]] void print_task(void* pvParams) {
  send_note(spkr_channel, 900, 200, 250);
  send_note(spkr_channel, 900, 200, 250);
  ledcWriteTone(spkr_channel, 0);
  while(true) {
    char buf[128];
    sprintf(buf, "Free heap: %lu, Free PSRAM: %lu", ESP.getFreeHeap(), ESP.getFreePsram());
    float n2 = Sensors::read_n2_rpm();
    float n3 = Sensors::read_n3_rpm();
    //ledcWriteTone(spkr_channel, (n2+n3)/2);
    Serial.println(buf);
    Serial.print("N2: ");
    Serial.print(n2);
    Serial.print(" N3: ");
    Serial.print(n3);
    Serial.print(" ATF: ");
    Serial.print(Sensors::read_atf_temp());
    Serial.print(" LOCK: ");
    Serial.println(Sensors::read_park_lock());
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void test_can(void* args) {
  while(true) {
      egs52_can_handler->set_display_speed_step(SpeedStep::P);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::R);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::N);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::D);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::A);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::F);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::ONE);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::TWO);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::THREE);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::FOUR);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::FIVE);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::SIX);
      vTaskDelay(250);
      egs52_can_handler->set_display_speed_step(SpeedStep::SEVEN);
      vTaskDelay(250);

      egs52_can_handler->set_drive_profile(DriveProfileDisplay::Agility);
      vTaskDelay(500);
      egs52_can_handler->set_drive_profile(DriveProfileDisplay::Winter);
      vTaskDelay(500);
      egs52_can_handler->set_drive_profile(DriveProfileDisplay::Manual);
      vTaskDelay(500);
      egs52_can_handler->set_drive_profile(DriveProfileDisplay::Standard);
      vTaskDelay(500);
      egs52_can_handler->set_drive_profile(DriveProfileDisplay::FAILURE);
      vTaskDelay(500);
      egs52_can_handler->set_drive_profile(DriveProfileDisplay::Comfort);
      vTaskDelay(500);

  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SPKR, OUTPUT);

  ledcSetup(spkr_channel, 2000, 8);
  ledcAttachPin(PIN_SPKR, spkr_channel);
  //ledcWriteTone(spkr_channel, 500);

  // Setup CAN so car is happy TCM is alive!
  egs52_can_handler = new Egs52Can();
  egs52_can_handler->start_tx_rx_loop();

  Sensors::configure_sensor_pins();
  xTaskCreate(&print_task, "printer", 4096, nullptr, 5, nullptr);
  xTaskCreate(&test_can, "printer", 1024, nullptr, 10, nullptr);

  egs52_can_handler->set_actual_gear(Gear::P);
  egs52_can_handler->set_shifter_possition(ShifterPosition::P);
  egs52_can_handler->set_drive_profile(DriveProfileDisplay::Comfort);
  egs52_can_handler->set_display_message(DisplayMessage::None);
  egs52_can_handler->set_target_gear(Gear::P);
  egs52_can_handler->set_actual_gear(Gear::P);
}


// Don't add any code here. Use RTOSTasks only
// This loop just idles forever
void loop() { vTaskDelay(10000); }