#include "solenoids.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "pins.h"

esp_adc_cal_characteristics_t adc1_cal;
bool all_calibrated = false;

Solenoid::Solenoid(const char *name, gpio_num_t pwm_pin, FeedbackConfig f_config, uint32_t frequency, uint16_t start_pwm, ledc_channel_t channel, ledc_timer_t timer)
{
    this->channel = channel;
    this->timer = timer;
    this->feedback_cfg = f_config;
    this->ready = true; // Assume ready unless error!
    this->name = name;
    this->current = 0;
    this->current_mutex = portMUX_INITIALIZER_UNLOCKED;
    this->vref = 0;
    this->vref_calibrated = false;

    // Set GPIO direction of feedback pin
    esp_err_t res = gpio_set_direction(f_config.pin, GPIO_MODE_INPUT);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOGE("SOLENOID", "Solenoid %s failed to set feedback pin GPIO mode to GPIO_MODE_INPUT. Status code %d!", name, res);
        return;
    }

    // Set attenuation of feedback pin to 11db
    res = adc1_config_channel_atten(f_config.channel, ADC_ATTEN_11db);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOGE("SOLENOID", "Solenoid %s failed to set feedback pin attenuation. Status code %d!", name, res);
        return;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE, // Low speed timer mode
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = timer,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK};
    // Set the timer configuration
    res = ledc_timer_config(&timer_cfg);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOGE("SOLENOID", "Solenoid %s timer init failed. Status code %d!", name, res);
        return;
    }

    // Set PWM channel configuration
    ledc_channel_config_t channel_cfg = {
        .gpio_num = pwm_pin,
        .speed_mode = ledc_mode_t::LEDC_HIGH_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, // Disable fade interrupt
        .timer_sel = timer,
        .duty = start_pwm,
        .hpoint = 0};

    res = ledc_channel_config(&channel_cfg);
    if (res != ESP_OK)
    {
        this->ready = false;
        ESP_LOGE("SOLENOID", "Solenoid %s channel init failed. Status code %d!", name, res);
        return;
    }
    ESP_LOGI("SOLENOID", "Solenoid %s init OK!", name);
}

void Solenoid::write_pwm(uint8_t pwm)
{
    esp_err_t res = ledc_set_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel, (uint32_t)pwm << 4); // Convert from 8bit to 12bit
    res = ledc_update_duty(ledc_mode_t::LEDC_HIGH_SPEED_MODE, this->channel);
    if (res == ESP_OK)
    {
        this->curr_pwm = pwm;
    }
    else
    {
        ESP_LOGE("SOLENOID", "Solenoid %s failed to set duty to %d!", name, pwm);
    }
}

uint16_t Solenoid::get_vref() {
    return this->vref;
}

void Solenoid::write_pwm_percent(uint16_t percent) {
    uint32_t clamped = (percent > 1000) ? 1000 : percent;
    uint32_t request = (255 * clamped) / 1000;
    this->write_pwm(request);
}

uint16_t Solenoid::get_pwm()
{
    return this->curr_pwm;
}

uint16_t Solenoid::get_current_estimate()
{
    portENTER_CRITICAL(&this->current_mutex);
    uint16_t r = this->current;
    portEXIT_CRITICAL(&this->current_mutex);

    /**
     * Calibration data from ADC:
     * 65300 - 3180mV
     * 0 - 0mV
     * 
     * With 0.005Ohm shunt resistor and INA180A3 amplifier:
     * 300mV = 6000mA
     * 150mV = 3000mA
     * 0mV = 0mA
     */

    int current = 0;
    // Convert ADC reading to approx mV
    // Voltage = ADC_READING * 0.0487
    // Current = Voltage * 2
    //
    // So basically current = ADC_READING * 0.0974
    if (this->vref_calibrated) {
        current = (r-this->vref)*0.0974;
    } else {
        current = r*0.0974;
    }
    return current;
}

void Solenoid::__set_current_internal(uint16_t c)
{
    portENTER_CRITICAL(&this->current_mutex);
    this->current = c;
    portEXIT_CRITICAL(&this->current_mutex);
}

void Solenoid::__set_vref(uint16_t ref)
{
    this->vref = ref;
    this->vref_calibrated = true;
}

bool Solenoid::init_ok()
{
    return this->ready;
}

Solenoid *sol_y3 = nullptr;
Solenoid *sol_y4 = nullptr;
Solenoid *sol_y5 = nullptr;

Solenoid *sol_mpc = nullptr;
Solenoid *sol_spc = nullptr;
Solenoid *sol_tcc = nullptr;

#define BYTES_PER_SAMPLE 2
#define NUM_SAMPLES 1024

char dma_buffer[BYTES_PER_SAMPLE*NUM_SAMPLES];


const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 200000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = NUM_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = false
};

void read_solenoids_i2s(void*) {
    esp_log_level_set("I2S", esp_log_level_t::ESP_LOG_WARN); // Discard noisy I2S logs!
    // Y3, Y4, Y5, MPC, SPC, TCC
    adc1_channel_t solenoid_channels[6] = { ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_7, ADC1_CHANNEL_6, ADC1_CHANNEL_4, ADC1_CHANNEL_5 };
    Solenoid* sol_order[6] = { sol_y3, sol_y4, sol_y5, sol_mpc, sol_spc, sol_tcc };
    uint8_t solenoid_id = 0;
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
    //SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
    size_t bytes_read = 0;
    #define SAMPLE_COUNT 5
    uint32_t samples[SAMPLE_COUNT];
    uint32_t sample_id=0;
    uint32_t avg = 0;
    while(true) {
        i2s_set_adc_mode(ADC_UNIT_1, solenoid_channels[solenoid_id]);
        i2s_adc_enable(I2S_NUM_0);
        if (all_calibrated) {
            vTaskDelay(33 / portTICK_RATE_MS); // Approx 5 refreshes per second
        }
        sample_id = 0;
        avg = 0;
        while(sample_id < SAMPLE_COUNT) {
            bytes_read = 0;
            i2s_read(I2S_NUM_0, &dma_buffer, NUM_SAMPLES*BYTES_PER_SAMPLE, &bytes_read, portMAX_DELAY);
            uint32_t tmp = 0;
            for (int i = 0; i < BYTES_PER_SAMPLE*NUM_SAMPLES; i += BYTES_PER_SAMPLE) {
                tmp += (uint32_t)(dma_buffer[i] << 8 | dma_buffer[i+1]) & 0x00000000FFFFFFFF;
            }
            samples[sample_id] = tmp / NUM_SAMPLES;
            sample_id += 1;
        }
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            avg += samples[i];
        }
        avg /= SAMPLE_COUNT;
        sol_order[solenoid_id]->__set_current_internal(avg);
        if (!all_calibrated) {
            sol_order[solenoid_id]->__set_vref(avg);
        }
        solenoid_id++;
        if (solenoid_id == 6) {
            solenoid_id = 0;
            all_calibrated = true;
        }
        // Configure ADC again and loop to read the next solenoid!
        i2s_adc_disable(I2S_NUM_0);
    }
}

bool init_all_solenoids()
{
    // Need to configure ADC1
    esp_err_t res = adc1_config_width(ADC_WIDTH_12Bit);
    if (res != ESP_OK)
    {
        ESP_LOGE("SOLENOID", "FATAL. Could not configure ADC1 width!");
        return false;
    }

    // Get calibration from ESP32 for ADC1
    esp_adc_cal_value_t type = esp_adc_cal_characterize(adc_unit_t::ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 0, &adc1_cal);
    switch (type)
    {
    case esp_adc_cal_value_t::ESP_ADC_CAL_VAL_EFUSE_VREF:
        ESP_LOGI("SOLENOID", "ESP_ADC_CAL_VAL_EFUSE_VREF used for ADC calibration!");
        break;
    case esp_adc_cal_value_t::ESP_ADC_CAL_VAL_EFUSE_TP:
        ESP_LOGI("SOLENOID", "ESP_ADC_CAL_VAL_EFUSE_TP used for ADC calibration!");
        break;
    case esp_adc_cal_value_t::ESP_ADC_CAL_VAL_DEFAULT_VREF:
        ESP_LOGI("SOLENOID", "ESP_ADC_CAL_VAL_DEFAULT_VREF used for ADC calibration!");
        break;
    default:
        break;
    }
    sol_y3 = new Solenoid("Y3", PIN_Y3_PWM, FeedbackConfig{PIN_Y3_SENSE, ADC1_CHANNEL_0, 0}, 1000, 0, ledc_channel_t::LEDC_CHANNEL_0, ledc_timer_t::LEDC_TIMER_0);
    sol_y4 = new Solenoid("Y4", PIN_Y4_PWM, FeedbackConfig{PIN_Y4_SENSE, ADC1_CHANNEL_3, 1}, 1000, 0, ledc_channel_t::LEDC_CHANNEL_1, ledc_timer_t::LEDC_TIMER_0);
    sol_y5 = new Solenoid("Y5", PIN_Y5_PWM, FeedbackConfig{PIN_Y5_SENSE, ADC1_CHANNEL_7, 2}, 1000, 0, ledc_channel_t::LEDC_CHANNEL_2, ledc_timer_t::LEDC_TIMER_0);
    sol_mpc = new Solenoid("MPC", PIN_MPC_PWM, FeedbackConfig{PIN_MPC_SENSE, ADC1_CHANNEL_6, 3}, 1000, 0, ledc_channel_t::LEDC_CHANNEL_3, ledc_timer_t::LEDC_TIMER_1);
    sol_spc = new Solenoid("SPC", PIN_SPC_PWM, FeedbackConfig{PIN_SPC_SENSE, ADC1_CHANNEL_4, 4}, 1000, 0, ledc_channel_t::LEDC_CHANNEL_4, ledc_timer_t::LEDC_TIMER_1);
    sol_tcc = new Solenoid("TCC", PIN_TCC_PWM, FeedbackConfig{PIN_TCC_SENSE, ADC1_CHANNEL_5, 5}, 100, 0, ledc_channel_t::LEDC_CHANNEL_5, ledc_timer_t::LEDC_TIMER_2);

    res = ledc_fade_func_install(0);
    if (res != ESP_OK) {
        ESP_LOGE("SOLENOID", "FATAL. Could not load insert LEDC fade function %s", esp_err_to_name(res));
        return false;
    }

    bool ok = sol_tcc->init_ok() && sol_mpc->init_ok() && sol_spc->init_ok() && sol_y3->init_ok() && sol_y4->init_ok() && sol_y5->init_ok();

    if (!ok) { // Init error, don't do anything else
        return false;
    }

    xTaskCreate(read_solenoids_i2s, "I2S-Reader", 8192, nullptr, 1, nullptr);
    
    while(!all_calibrated) {
        vTaskDelay(2);
    }
    ESP_LOGI("SOLENOID", 
        "Solenoid calibration readings: Y3: %d, Y4: %d, Y5: %d, MPC: %d, SPC: %d, TCC: %d",
            sol_y3->get_vref(),
            sol_y4->get_vref(),
            sol_y5->get_vref(),
            sol_mpc->get_vref(),
            sol_spc->get_vref(),
            sol_tcc->get_vref()
    );
    return ok;
}