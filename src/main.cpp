#include <pins.h>
#include <sensors.h>
#include <config.h>
#include <canbus/kwp_server.h>
#include <canbus/egs52_can.h>
#include <driver/ledc.h>

#include <esp_freertos_hooks.h>

#include <pwm_channels/channels.h>


static volatile uint64_t ticks_core0 = 0;
static volatile uint64_t ticks_core1 = 0;

int spkr_channel = 0;


void send_note(uint8_t channel, uint32_t freq, uint32_t dur, uint32_t total_dur) {
  spkr_pwm.change_freq(freq);
  spkr_pwm.write_pwm(128);
  vTaskDelay(dur / portTICK_PERIOD_MS);
  spkr_pwm.write_pwm(0);
  vTaskDelay((total_dur - dur) / portTICK_PERIOD_MS);
}

[[noreturn]] void print_task(void* pvParams) {
  send_note(spkr_channel, 900, 200, 250);
  send_note(spkr_channel, 900, 200, 250);
  while(true) {
    /*
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
    */
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
/*
void setup() {
  //Serial.begin(115200);
  gpio_set_direction(PIN_SPKR, gpio_mode_t::GPIO_MODE_OUTPUT);

  ledcSetup(spkr_channel, 2000, 8);
  ledcAttachPin(PIN_SPKR, spkr_channel);

  // Setup CAN so car is happy TCM is alive!
  egs52_can_handler = new Egs52Can();
  egs52_can_handler->start_tx_rx_loop();
  xTaskCreate(&print_task, "printer", 4096, nullptr, 5, nullptr);
  xTaskCreate(&test_can, "CAN_TEST", 1024, nullptr, 10, nullptr);
  Sensors::configure_sensor_pins();
}
*/

// Don't add any code here. Use RTOSTasks only
// This loop just idles forever
extern "C" void app_main(void) {
  egs52_can_handler = new Egs52Can();
  egs52_can_handler->start_tx_rx_loop();
  xTaskCreate(&print_task, "printer", 4096, nullptr, 5, nullptr);
  xTaskCreate(&test_can, "CAN_TEST", 1024, nullptr, 10, nullptr);
  Sensors::configure_sensor_pins();
}