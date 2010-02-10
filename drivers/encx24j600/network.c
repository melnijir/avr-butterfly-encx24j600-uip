#include "global-conf.h"
#include "enc424j600.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LCD_driver.h"

extern MAC_ADDR mac_addr;
extern u8_t uip_buf[UIP_BUFSIZE + 2];
extern u16_t uip_len;

unsigned int network_read(void) {
    uint16_t len;
    len = enc424j600PacketReceive(UIP_BUFSIZE, (u8_t *) uip_buf);
    return len;
}

void network_send(void) {
    enc424j600PacketSend(uip_len, (u8_t *) uip_buf);
}

void network_init(void) {
    //Initialise the device
    enc424j600Init();
}

void network_get_MAC(u08* macaddr) {
    // read MAC address registers
    // NOTE: MAC address in ENC28J60 is byte-backward
    *macaddr++ = mac_addr.v[0];
    *macaddr++ = mac_addr.v[1];
    *macaddr++ = mac_addr.v[2];
    *macaddr++ = mac_addr.v[3];
    *macaddr++ = mac_addr.v[4];
    *macaddr++ = mac_addr.v[5];
}

void network_set_MAC(u08* macaddr) {
    // write MAC address
    // NOTE: MAC address in ENC28J60 is byte-backward

}

