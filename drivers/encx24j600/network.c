#include "global-conf.h"
#include "enc424j600.h"
#include <avr/io.h>
#include <util/delay.h>
#include "LCD_driver.h"
#include "../interfaces/network.h"

extern u8_t uip_buf[UIP_BUFSIZE + 2];

unsigned int network_read(void) {
    //Receive pacet
    uint16_t len;
    len = enc424j600PacketReceive(UIP_BUFSIZE, (u8_t *) uip_buf);
    return len;
}

void network_send(void) {
    //Send packet
    enc424j600PacketSend(uip_len, (u8_t *) uip_buf);
}

void network_init(void) {
    //Initialise the device
    enc424j600Init();
}

void network_get_MAC(u8_t* mac_addr) {
    // read MAC address registers
    enc424j600ReadMacAddr(mac_addr);
}

void network_set_MAC(u8_t* mac_addr) {
    // write MAC address
    enc424j600SetMacAddr(mac_addr);
}

void network_set_sleep(void) {
    // sleep :)
    enc424j600PowerSaveEnable();
}

void network_set_wake_up(void) {
    // sleep :)
    enc424j600PowerSaveDisable();
}

