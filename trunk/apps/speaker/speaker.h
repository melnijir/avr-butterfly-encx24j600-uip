#ifndef __SIMPLE_HTTPD_H__
#define __SIMPLE_HTTPD_H__
#include "uipopt.h"
#include "psock.h"

typedef struct speaker_state {
    struct psock p;
    char inputbuffer[10];
    char name[10];
} uip_tcp_appstate_t;

void speaker_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL speaker_appcall
#endif /* UIP_APPCALL */

void speaker_init(void);

#endif /* __SIMPLE_HTTPD_H__ */
