#include <pins.h>
#include <sensors.h>
#include <config.h>
#include <canbus/kwp_server.h>
#include <canbus/egs52_can.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_freertos_hooks.h>
#include <pwm_channels/channels.h>

int spkr_channel = 0;


void send_note(uint8_t channel, uint32_t freq, uint32_t dur, uint32_t total_dur) {
  spkr_pwm.change_freq(freq);
  spkr_pwm.write_pwm(128);
  vTaskDelay(dur / portTICK_PERIOD_MS);
  spkr_pwm.write_pwm(0);
  vTaskDelay((total_dur - dur) / portTICK_PERIOD_MS);
}

uint32_t v;

[[noreturn]] void print_task(void* pvParams) {
  send_note(spkr_channel, 900, 200, 250);
  send_note(spkr_channel, 900, 200, 250);
  bool haveZero = true;
  esp_chip_info_t info;
  esp_chip_info(&info);
  while(true) {
    uint32_t n2 = Sensors::read_n2_rpm();
    uint32_t n3 = Sensors::read_n3_rpm();
    ESP_LOGD("PRINT", "N2 RPM: %u, N3 RPM: %u, [%d %d %d | %d %d %d]", n2, n3, spc_pwm.get_pwm(), mpc_pwm.get_pwm(), tcc_pwm.get_pwm(), y3_pwm.get_pwm(), y4_pwm.get_pwm(), y5_pwm.get_pwm());
    //if (n2 != 0 || n3 != 0 ) {
    //  spkr_pwm.change_freq((n2+1 + n3+1) / 2);
    //  spkr_pwm.write_pwm(128);
    //  haveZero = false;
    //} else if (!haveZero) {
    //  spkr_pwm.write_pwm(0);
    //  haveZero = true;
    //}
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}

void test_can(void* args) {
  while(Sensors::read_vbatt() < 700) {
    vTaskDelay(10);
  }
  egs52_can_handler->set_drive_profile(DriveProfileDisplay::Comfort);
  Gear target = Gear::P;
  Gear actual = Gear::P;
  bool idling = false;
  bool garageshifting = false;
  uint64_t last_garage_shift = esp_timer_get_time() / 1000;
  bool into_reverse = false;
  while(true) {
    ShifterPosition pos = egs52_can_handler->get_shifter_position();
    egs52_can_handler->set_shifter_possition(pos);
    uint32_t n2 = Sensors::read_n2_rpm();
    uint32_t n3 = Sensors::read_n3_rpm();
    uint32_t rpm = 0;
    if (n2 > n3) { rpm = n2; } else { rpm = n3; }
    egs52_can_handler->set_turbine_rpm(rpm);
    switch(pos) {
      case ShifterPosition::P:
        egs52_can_handler->set_display_speed_step(SpeedStep::P);
        egs52_can_handler->set_is_safe_start(true);
        break;
      case ShifterPosition::P_R:
      case ShifterPosition::R:
        
        egs52_can_handler->set_display_speed_step(SpeedStep::R);
        egs52_can_handler->set_is_safe_start(false);
        break;
      case ShifterPosition::R_N:
      case ShifterPosition::N:
        egs52_can_handler->set_display_speed_step(SpeedStep::N);
        egs52_can_handler->set_is_safe_start(true);
        break;
      case ShifterPosition::N_D:
      case ShifterPosition::PLUS:
      case ShifterPosition::MINUS:
      case ShifterPosition::D:
        egs52_can_handler->set_display_speed_step(SpeedStep::D);
        egs52_can_handler->set_is_safe_start(false);
        break;
      default:
        break;
    }
    if ((pos == ShifterPosition::P || pos == ShifterPosition::N) && Sensors::read_park_lock() ) {
      // We are in park or neutral!
      if (!idling) {
        ESP_LOGD("GB","Idling!");
      }
      if (pos == ShifterPosition::P) {
        actual = Gear::P;
        target = Gear::P;
      } else {
        actual = Gear::N;
        target = Gear::P;
      }
      mpc_pwm.write_pwm(102);
      spc_pwm.write_pwm(84);
      y4_pwm.write_pwm(128);
      idling = true;
      garageshifting = false;
    } else if ((pos == ShifterPosition::P_R || pos == ShifterPosition::R_N || pos == ShifterPosition::N_D) && !Sensors::read_park_lock()) {
      if (!garageshifting) {
        ESP_LOGD("GB","Garageshifting!");
      }
      y4_pwm.write_pwm(255);
      if (pos == ShifterPosition::P_R) {
        y3_pwm.write_pwm(255);
      }
      mpc_pwm.write_pwm(84);
      spc_pwm.write_pwm(120);
      idling = false;
      garageshifting = true;
      last_garage_shift = esp_timer_get_time() / 1000;
    } else {
      idling = false;
    }
    if (garageshifting && (esp_timer_get_time() / 1000 - last_garage_shift) > 1000) {
      ESP_LOGD("GB","Garageshifting done!");
      garageshifting = false;
      spc_pwm.write_pwm(0);
      mpc_pwm.write_pwm(0);
      y4_pwm.write_pwm(0);
      y3_pwm.write_pwm(0);
      if (Sensors::read_n2_rpm() == 0 && Sensors::read_n3_rpm() == 0) {
        if (into_reverse) {
          actual = Gear::R1;
          target = Gear::R1;
        } else {
          actual = Gear::D1;
          target = Gear::D1;
        }
      }
    }
    vTaskDelay(10);
    egs52_can_handler->set_target_gear(target);
    egs52_can_handler->set_actual_gear(actual);
  }
  vTaskDelete(NULL);
 //while(true) {
 //   tcc_pwm.fade_pwm(0, 1000);
 //   vTaskDelay(1000);
 //   tcc_pwm.fade_pwm(32, 500);
 //   vTaskDelay(700);
 //}
}

// Don't add any code here. Use RTOSTasks only
// This loop just idles forever
extern "C" void app_main(void) {
  egs52_can_handler = new Egs52Can();
  egs52_can_handler->start_tx_rx_loop();
  Sensors::configure_sensor_pins();
  xTaskCreate(&print_task, "printer", 4096, nullptr, 5, nullptr);
  xTaskCreate(&test_can, "CAN_TEST", 2048, nullptr, 10, nullptr);
  xTaskCreate(&pwm_solenoid_fader_thread, "SOL_FADE", 1024, nullptr, 5, nullptr);
}