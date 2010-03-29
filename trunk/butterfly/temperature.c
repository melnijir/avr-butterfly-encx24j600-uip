#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/iom169.h>
#include "uip-conf.h"
#include "avrlibdefs.h"
#include "temperature.h"


const prog_uint16_t TEMP_Celsius_pos[] = {// Positive Celsius temps (ADC-value) for 0 to 60 degrees C
    806, 796, 786, 775, 765, 754, 743, 732, 720, 709, 697, 685, 673, 661, 649,
    636, 624, 611, 599, 586, 574, 562, 549, 537, 524, 512, 500, 488, 476, 464,
    452, 440, 429, 418, 406, 396, 385, 374, 364, 354, 344, 334, 324, 315, 306,
    297, 288, 279, 271, 263, 255, 247, 240, 233, 225, 219, 212, 205, 199, 193,
    187,
};

const prog_uint16_t TEMP_Celsius_neg[] = {// Negative Celsius temps (ADC-value) from -1 to -15 degrees C
    815, 825, 834, 843, 851, 860, 868, 876, 883, 891, 898, 904, 911, 917, 923,
};

s08 map_to_C(u16_t a2d) {
    u8_t v = 0;
    if (a2d > 810) { // If it's a negative temperature
        for (u8_t i = 0; i <= 25; i++) { // Find the temperature
            if ((prog_uint16_t) a2d <= pgm_read_word_near(TEMP_Celsius_neg + i)) {
                v = 0 - i; // Make it negative
                break;
            }
        }
    } else if (a2d < 800) { // If it's a positive temperature
        for (u8_t i = 0; i < 100; i++) {
            if ((prog_uint16_t) a2d >= pgm_read_word_near(TEMP_Celsius_pos + i)) {
                v = i;
                break;
            }
        }
    }
    return v;
}

void temperature_init() {
    //Start ADC converter
    ADMUX = 0; // select channel
    sbi(ADMUX, REFS0); // use Vcc as reference voltage

    sbi(ADCSRA, ADPS0); // divide clock for accurate reading
    sbi(ADCSRA, ADPS1); // divide clock for accurate reading
    sbi(ADCSRA, ADPS2); // divide clock for accurate reading
}

s08 get_temp() {
    u16_t temp;
    sbi(ADCSRA, ADEN); // enable converter
    sbi(ADCSRA, ADSC); // start conversion
    while ((ADCSRA & 0x40) > 0); // wait until conversion is complete
    uint8_t lowByte = ADCL;
    uint8_t highByte = ADCH;
    cbi(ADCSRA, ADEN); // disable converter
    temp = ((highByte << 8) + lowByte);
    return map_to_C(temp);
}