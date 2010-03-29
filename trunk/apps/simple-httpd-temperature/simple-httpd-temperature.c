#include "uip.h"
#include <string.h>
#include <stdio.h>
#include "simple-httpd-temperature.h"

//Temp inludes
#include "temperature.h"
#include "LCD_driver.h"
#include "enc424j600.h"



static int handle_connection(struct simple_httpd_state *s);

void simple_httpd_init(void) {
    uip_listen(HTONS(80));
    
    temperature_init();
}

void simple_httpd_appcall(void) {
    struct simple_httpd_state *s = &(uip_conn->appstate);

    if (uip_connected()) {
        PSOCK_INIT(&s->p, NULL, 0);
    }

    handle_connection(s);
}

static int handle_connection(struct simple_httpd_state *s) {
    s08 temp = get_temp();
    s08 temp_text[4];
    itoa(temp,temp_text);

    enc424j600SaveToUserSpace(4091,temp_text,sizeof(temp_text));
    temp_text[0] = 0;
    temp_text[1] = 0;
    temp_text[2] = 0;
    temp_text[3] = 0;
    enc424j600ReadFromUserSpace(4091,temp_text,sizeof(temp_text));


    PSOCK_BEGIN(&s->p);
    PSOCK_SEND_STR(&s->p, "HTTP/1.0 200 OK\r\n");
    PSOCK_SEND_STR(&s->p, "Content-Type: text/html\r\n");
    PSOCK_SEND_STR(&s->p, "\r\n");
    PSOCK_SEND_STR(&s->p, "<h1>Temperature sensor</h1><p>Actual temp: ");
    PSOCK_SEND_STR(&s->p, temp_text);
    PSOCK_SEND_STR(&s->p, "&deg;C</p>");
    PSOCK_SEND_STR(&s->p, "<p>Mem free: ");
    u8_t free = StackCount();
    itoa(free,temp_text);
    PSOCK_SEND_STR(&s->p, temp_text);
    PSOCK_SEND_STR(&s->p, "bytes</p>");
    PSOCK_CLOSE(&s->p);
    PSOCK_END(&s->p);
}
