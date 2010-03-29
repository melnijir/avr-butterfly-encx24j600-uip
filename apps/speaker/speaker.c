#include "uip.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "speaker.h"
//#include "drivers/encx24j600/avrlibdefs.h"

#include <avr/io.h>
#include <avr/iom169.h>
#include <avr/interrupt.h>
#include "avrlibdefs.h"
#include "global-conf.h"
#include "avrlibtypes.h"
#include <util/delay.h>

static int handle_connection(struct speaker_state *s);
static u16 tone = 0;
static u16 clock;

void speaker_init(void) {
    uip_listen(HTONS(23));

    /*Speaker test interrupt*/
    cli();
    //Activate overflow interrupt for timer0
    TIMSK2 |= (1 << TOIE2);
    //Use prescaler 1024
    TCCR2A |= ((1 << CS20));
    //Activate interrupts
    sei();
}

ISR(TIMER2_COMP_vect) {
    clock %= 100;
    if (clock++ < tone) {
        sbi(DDRB, PB5);
        sbi(PORTB, PB5);
        _delay_us(60);
        cbi(PORTB, PB5);
    }
    TIFR2 |= (1 << TOV2);
}

void speaker_appcall(void) {
    struct speaker_state *s = &(uip_conn->appstate);

    if (uip_connected()) {
        PSOCK_INIT(&s->p, s->inputbuffer, sizeof (s->inputbuffer));
    }

    handle_connection(s);
}

static int handle_connection(struct speaker_state *s) {
    /*Speaker test*/
    PSOCK_BEGIN(&s->p);

    while (1) {
        PSOCK_SEND_STR(&s->p, "Give a tone:\n");
        PSOCK_READTO(&s->p, '\n');
        strncpy(s->name, s->inputbuffer, sizeof (s->name));
        tone = atoi(s->name);
        if (tone == 999) {
            PSOCK_CLOSE(&s->p);
            break;
        }
    }

    tone = 0;
    PSOCK_END(&s->p);
}
