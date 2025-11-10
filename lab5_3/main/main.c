/* SPDX-License-Identifier: Apache-2.0 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "MORSE_RX";

#if CONFIG_IDF_TARGET_ESP32
#define ADC_CHAN ADC_CHANNEL_5
#else
#define ADC_CHAN ADC_CHANNEL_3
#endif

#define ADC_ATTEN ADC_ATTEN_DB_12
#define SAMPLE_MS 10   // sampling interval

// Morse code table
typedef struct { const char *morse; char letter; } MorseMap;
MorseMap morse_table[] = {
    {".-", 'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'}, {".", 'E'},
    {"..-.", 'F'}, {"--.", 'G'}, {"....", 'H'}, {"..", 'I'}, {".---", 'J'},
    {"-.-", 'K'}, {".-..", 'L'}, {"--", 'M'}, {"-.", 'N'}, {"---", 'O'},
    {".--.", 'P'}, {"--.-", 'Q'}, {".-.", 'R'}, {"...", 'S'}, {"-", 'T'},
    {"..-", 'U'}, {"...-", 'V'}, {".--", 'W'}, {"-..-", 'X'}, {"-.--", 'Y'},
    {"--..", 'Z'}, {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'},
    {"....-", '4'}, {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'},
    {"----.", '9'}
};

// Decode Morse string to char
char morse_to_char(const char *code) {
    for(int i=0;i<sizeof(morse_table)/sizeof(MorseMap);i++)
        if(strcmp(morse_table[i].morse, code)==0) return morse_table[i].letter;
    return '?';
}

// ADC calibration helpers
static bool adc_calib_init(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten, adc_cali_handle_t *handle) {
    adc_cali_handle_t h = NULL; bool ok=false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {unit, chan, atten, ADC_BITWIDTH_DEFAULT};
    if(adc_cali_create_scheme_curve_fitting(&cfg, &h)==ESP_OK) ok=true;
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg2 = {unit, chan, atten, ADC_BITWIDTH_DEFAULT};
    if(!ok && adc_cali_create_scheme_line_fitting(&cfg2, &h)==ESP_OK) ok=true;
#endif
    *handle=h; return ok;
}

static void adc_calib_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void app_main(void) {
    // --- ADC init ---
    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t cfg1={ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg1, &adc1));
    adc_oneshot_chan_cfg_t ch_cfg={ADC_ATTEN, ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CHAN, &ch_cfg));

    adc_cali_handle_t cal_handle=NULL;
    bool do_calib=adc_calib_init(ADC_UNIT_1, ADC_CHAN, ADC_ATTEN, &cal_handle);

    int adc_raw=0, volt=0;

    // --- Manual threshold and DOT ---
    const int threshold = 40;     // mV, midway between LED off (~10mV) and LED on (~100mV)
    const int hysteresis = 5;     // mV, prevents bouncing
    const int dot = 50;           // ms, matches your Python DOT=0.2s

    ESP_LOGI(TAG,"Morse receiver started. Threshold=%d mV, DOT=%d ms", threshold, dot);

    // --- Buffers ---
    char letter_buf[16]; int lidx=0;
    char word_buf[64]; int widx=0;

    enum {OFF, ON} state = OFF;
    int duration = 0;

    while(1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1, ADC_CHAN, &adc_raw));
        if(do_calib) ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cal_handle, adc_raw, &volt));

        // --- Apply hysteresis ---
        bool led_on = (state == OFF) ? (volt > threshold + hysteresis) : (volt > threshold - hysteresis);

        if(led_on) { // LED ON
            if(state==OFF) {
                // OFF -> ON transition: check gaps
                if(duration >= 5*dot) { // word gap
                    if(lidx>0){
                        letter_buf[lidx]='\0';
                        char c = morse_to_char(letter_buf);
                        word_buf[widx++] = c;
                        lidx=0;
                    }
                    if(widx>0){
                        word_buf[widx]='\0';
                        ESP_LOGI(TAG,"Decoded Word: %s", word_buf);
                        widx=0;
                    }
                } else if(duration >= 2*dot) { // letter gap
                    if(lidx>0){
                        letter_buf[lidx]='\0';
                        char c = morse_to_char(letter_buf);
                        word_buf[widx++] = c;
                        lidx=0;
                    }
                }
                duration=0;
            }
            state=ON;
            duration+=SAMPLE_MS;
        } else { // LED OFF
            if(state==ON) {
                // ON -> OFF: record DOT or DASH
                letter_buf[lidx++] = (duration < 2*dot) ? '.' : '-';
                duration=0;
            }
            state=OFF;
            duration+=SAMPLE_MS;
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1));
    if(do_calib) adc_calib_deinit(cal_handle);
}


