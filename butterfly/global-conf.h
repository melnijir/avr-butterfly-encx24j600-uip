//Project specific configurations
#ifndef __GLOBAL_CONF_H__
#define __GLOBAL_CONF_H__

//Define frequency
#define F_CPU 8000000UL

// SPI port
#define SPI_PORT                    	PORTB
#define SPI_DDR                         DDRB
#define SPI_SCK                         PB1
#define SPI_MOSI                	PB2
#define SPI_MISO                	PB3
#define SPI_SS                          PB0
// control port
#define CONTROL_PORT                    PORTB
#define CONTROL_DDR                     DDRB
#define CONTROL_CS                  	PB0

//Include uip.h gives all the uip configurations in uip-conf.h
#include "uip.h"

#endif /*__GLOBAL_CONF_H__*/
