/*! \file ENC424J600conf.h \brief Microchip ENC424J600 Ethernet Interface Driver Configuration. */
//*****************************************************************************
//
// File Name	: 'ENC424J600conf.h'
// Title		: Microchip ENC424J600 Ethernet Interface Driver Configuration
// Author		: Pascal Stang
// Created		: 10/5/2004
// Revised		: 8/22/2005
// Version		: 0.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// Description	: This driver provides initialization and transmit/receive
//		functions for the ENC424J600 10Mb Ethernet Controller and PHY.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************


/* USERS NOTE:
 * Do not enter your hardware specific configurations here. Copy the settings
 * below and define them in your baord specific directory. Remember to also
 * define ENC424J600CONF_H so that these settings will not overwrite yours.
 * By modifying this in your board specific directory, you should be able to
 * update/upgrade your avr-uip distribution without having to modify updated
 * files.
 *
 *
 * DEVELOPERS NOTE:
 * Settings entered should be something rather common, and not update too often.
 */


#ifndef ENC424J600CONF_H
#define ENC424J600CONF_H

// ENC424J600 SPI port
#define ENC424J600_SPI_PORT		SPI_PORT
#define ENC424J600_SPI_DDR		SPI_DDR
#define ENC424J600_SPI_SCK		SPI_SCK
#define ENC424J600_SPI_MOSI		SPI_MOSI
#define ENC424J600_SPI_MISO		SPI_MISO
#define ENC424J600_SPI_SS		SPI_SS
// ENC424J600 control port
#define ENC424J600_CONTROL_PORT         CONTROL_PORT
#define ENC424J600_CONTROL_DDR          CONTROL_DDR
#define ENC424J600_CONTROL_CS		CONTROL_CS

// ENC424J600 config
#define RAMSIZE         		(0x6000)
#define TXSTART                         (0x0000)
#define RXSTART                         (0x0600)	// Should be an even memory address

#endif
