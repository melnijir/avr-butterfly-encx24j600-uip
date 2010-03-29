/* 
 * File:   main.c
 * Author: Jiri Melnikov
 *
 * Driver preview ENC424J600
 * IP ADRESS: 10.0.0.60
 * MASK: 255.255.255.0
 */

#include "global-conf.h"

#include <avr/io.h>
#include <util/delay.h>

#include "LCD_driver.h"
#include "uip_arp.h"
#include "network.h"
#include "enc424j600.h"
#include "avrlibdefs.h"

#include "timer.h"
#include "clock-arch.h"
#include "avrlibtypes.h"

#include "simple-httpd-temperature.h"

extern u8_t uip_buf[UIP_BUFSIZE + 2];
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

/*************************************
 * STACK SPACE FUNCTION
 * ***********************************/

#define STACK_CANARY 0xc5

extern uint8_t _end;
extern uint8_t __stack;

void StackPaint(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

void StackPaint(void)
{
#if 0
    uint8_t *p = &_end;

    while(p <= &__stack)
    {
        *p = STACK_CANARY;
        p++;
    }
#else
    __asm volatile ("    ldi r30,lo8(_end)\n"
                    "    ldi r31,hi8(_end)\n"
                    "    ldi r24,lo8(0xc5)\n" /* STACK_CANARY = 0xc5 */
                    "    ldi r25,hi8(__stack)\n"
                    "    rjmp .cmp\n"
                    ".loop:\n"
                    "    st Z+,r24\n"
                    ".cmp:\n"
                    "    cpi r30,lo8(__stack)\n"
                    "    cpc r31,r25\n"
                    "    brlo .loop\n"
                    "    breq .loop"::);
#endif
}

uint8_t StackCount(void)
{
    const uint8_t *p = &_end;
    uint8_t       c = 0;

    while(*p == STACK_CANARY && p <= &__stack) {
        p++;
        c++;
    }

    return c;
} 
/*************************************
 * END OF STACK SPACE FUNCTION
 * ***********************************/

int main(void) {
    //Enable interupts
    sei();

    //Init LCD
    LCD_Init();

    //Display welcome message
    LCD_puts_f(PSTR("START"));
    _delay_us(500000);

    //Init network
    network_init();

    //Display message about network initialization
    LCD_puts_f(PSTR("NET OK"));
    _delay_us(500000);

    //Basic static variables
    int i;
    uip_ipaddr_t ipaddr; //IP Address
    struct uip_eth_addr mac_addr; //MAC Address
    struct timer periodic_timer, arp_timer;

    //Init clock (timers)
    clock_init();

    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);

    //Read MAC from ethernet chip
    network_get_MAC((u8_t *) & mac_addr);

    /* Here can be specified own MAC address
    mac_addr.addr[0] = 0x10;
    mac_addr.addr[1] = 0x11;
    mac_addr.addr[2] = 0x12;
    mac_addr.addr[3] = 0x13;
    mac_addr.addr[4] = 0x14;
    mac_addr.addr[5] = 0x15;
    network_set_MAC((u8_t *) & mac_addr);*/

    //UIP initialization
    uip_init();
    //Set MAC address in UIP (not to network card, this is only for arp)
    uip_setethaddr(mac_addr);

    simple_httpd_init();

    uip_ipaddr(ipaddr, 10, 0, 0, 60);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 10, 0, 0, 138);
    uip_setdraddr(ipaddr);
    uip_ipaddr(ipaddr, 255, 255, 255, 0);
    uip_setnetmask(ipaddr);

    LCD_puts_f(PSTR("RUNNING"));
    _delay_us(500000);

    while (1) {
        uip_len = network_read();

        if (uip_len > 0) {
            if (BUF->type == htons(UIP_ETHTYPE_IP)) {
                uip_arp_ipin();
                uip_input();
                if (uip_len > 0) {
                    uip_arp_out();
                    network_send();
                }
            } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
                uip_arp_arpin();
                if (uip_len > 0) {
                    network_send();
                }
            }

        } else if (timer_expired(&periodic_timer)) {
            timer_reset(&periodic_timer);

            for (i = 0; i < UIP_CONNS; i++) {
                uip_periodic(i);
                if (uip_len > 0) {
                    uip_arp_out();
                    network_send();
                }
            }

            if (timer_expired(&arp_timer)) {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }


    }
    return 0;
}