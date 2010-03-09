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

extern u8_t uip_buf[UIP_BUFSIZE + 2];
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])


MAC_ADDR mac_addr;
IP_ADDR ip_addr;

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

    //Read MAC from ethernet chip
    network_get_MAC((u8_t *)&mac_addr);

    //Display message about network initialization
    LCD_puts_f(PSTR("NET OK"));
    _delay_us(500000);

    //Basic static variables
    int i;
    uip_ipaddr_t ipaddr;
    struct timer periodic_timer, arp_timer;

    //Init clock (timers)
    clock_init();

    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);

    //UIP initialization
    uip_init();

    struct uip_eth_addr mac = {{mac_addr.v[0], mac_addr.v[1], mac_addr.v[2], mac_addr.v[3], mac_addr.v[4], mac_addr.v[5]}};

    uip_setethaddr(mac);
    simple_httpd_init();

    uip_ipaddr(ipaddr, 10, 0, 0, 60);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 10, 0, 0, 138);
    uip_setdraddr(ipaddr);
    uip_ipaddr(ipaddr, 255, 255, 255, 0);
    uip_setnetmask(ipaddr);


    LCD_puts_f(PSTR("RUN OK"));
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