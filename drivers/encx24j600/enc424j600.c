/******************************************
 * Title        : Microchip ENCX24J600 Ethernet Interface Driver
 * Author       : Jiri Melnikov
 * Created      : 29.03.2010
 * Version      : 0.2
 * Target MCU   : Atmel AVR series
 *
 * Description  : * This driver provides initialization and transmit/receive
 *                  functions for the Microchip ENCX24J600 100Mb Ethernet
 *                  Controller and PHY.
 *                * As addition, userspace access and hardware checksum
 *                  functions are available.
 *                * Only supported interface is SPI, no PSP interface available
 *                  by now.
 *                * No security functions are supported by now.
 *
 *                * This driver is inspired by ENC28J60 driver from Pascal
 *                  Stang (2005).
 *
 *                * Some lines of code are rewritten from Microchip's TCP/IP
 *                  stack.
 * 
 * ****************************************/

#include "enc424j600.h"
#include "enc424j600conf.h"
#include "global.h"
#include <avr/io.h>
#include <util/delay.h>

// Binary constant identifiers for ReadMemoryWindow() and WriteMemoryWindow()
// functions
#define UDA_WINDOW		(0x1)
#define GP_WINDOW		(0x2)
#define RX_WINDOW		(0x4)

// Promiscuous mode, uncomment if you want to receive all packets, even those which are not for you
// #define PROMISCUOUS_MODE

// Auto answer to ICMP requests, uncomment if you want to ICMP echo packets be managed by hardware.
// (if enc424j600PacketReceive is called and ICMP packet is waiting, function returns null lenght and provides automatic response)
#define AUTO_ICMP_ECHO

// Hardware checksum computation
#define HARDWARE_CHECKSUM  //Comment to disable automatic hardware checksum in ip/icmp/tcp/udp packets
#ifdef HARDWARE_CHECKSUM

//#define HARDWARE_CHECKSUM_NULL  //Comment to disable cleraing checksums
#define IP_PROTOCOL1 0x08
#define IP_PROTOCOL2 0x00
#define ICMP_PROTOCOL 0x01
#define TCP_PROTOCOL 0x06
#define UDP_PROTOCOL 0x11
#define ETH_HEADER 14
#define IP_PROTOCOL_POS 23

#endif

// Internal MAC level variables and flags.
static u08 currentBank;
static u16 nextPacketPointer;

// Static functions
static void enc424j600SendSystemReset(void);

static bool enc424j600MACIsTxReady(void);
static void enc424j600MACFlush(void);
static u16 enc424j600ChecksumCalculation(u16 position, u16 length, u16 seed);

static void enc424j600WriteMemoryWindow(u08 window, u08 *data, u16 length);
static void enc424j600ReadMemoryWindow(u08 window, u08 *data, u16 length);

static u16 enc424j600ReadReg(u16 address);
static void enc424j600WriteReg(u16 address, u16 data);
static u16 enc424j600ReadPHYReg(u08 address);
static void enc424j600WritePHYReg(u08 address, u16 Data);
static void enc424j600ReadN(u08 op, u08* data, u16 dataLen);
static void enc424j600WriteN(u08 op, u08* data, u16 dataLen);
static void enc424j600BFSReg(u16 address, u16 bitMask);
static void enc424j600BFCReg(u16 address, u16 bitMask);

static void enc424j600ExecuteOp0(u08 op);
static u08 enc424j600ExecuteOp8(u08 op, u08 data);
static u16 enc424j600ExecuteOp16(u08 op, u16 data);
static u32 enc424j600ExecuteOp32(u08 op, u32 data);

/********************************************************************
 * INITIALIZATION
 * ******************************************************************/
void enc424j600Init(void) {
    //Set default bank
    currentBank = 0;

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);

    sbi(ENC424J600_CONTROL_DDR, ENC424J600_CONTROL_CS);
    sbi(ENC424J600_CONTROL_PORT, ENC424J600_CONTROL_CS);

    sbi(ENC424J600_SPI_PORT, ENC424J600_SPI_SCK); // set SCK hi
    sbi(ENC424J600_SPI_DDR, ENC424J600_SPI_SCK); // set SCK as output
    cbi(ENC424J600_SPI_DDR, ENC424J600_SPI_MISO); // set MISO as input
    sbi(ENC424J600_SPI_DDR, ENC424J600_SPI_MOSI); // set MOSI as output
    sbi(ENC424J600_SPI_DDR, ENC424J600_SPI_SS); // SS must be output for Master mode to work

    // initialize SPI interface
    // master mode
    sbi(SPCR, MSTR);
    // select clock phase positive-going in middle of data
    cbi(SPCR, CPOL);
    // Data order MSB first
    cbi(SPCR, DORD);
    // switch to f/4 2X = f/2 bitrate
    cbi(SPCR, SPR0);
    cbi(SPCR, SPR1);
    sbi(SPSR, SPI2X);
    // enable SPI
    sbi(SPCR, SPE);

    // Perform a reliable reset
    enc424j600SendSystemReset();

    // Initialize RX tracking variables and other control state flags
    nextPacketPointer = RXSTART;

    // Set up TX/RX/UDA buffer addresses
    enc424j600WriteReg(ETXST, TXSTART);
    enc424j600WriteReg(ERXST, RXSTART);
    enc424j600WriteReg(ERXTAIL, RAMSIZE - 2);
    enc424j600WriteReg(EUDAST, USSTART);
    enc424j600WriteReg(EUDAND, USEND);

    // If promiscuous mode is set, than allow accept all packets
#ifdef PROMISCUOUS_MODE
    enc424j600WriteReg(ERXFCON, (ERXFCON_CRCEN | ERXFCON_RUNTEN | ERXFCON_UCEN | ERXFCON_NOTMEEN | ERXFCON_MCEN));
#endif

    // Set PHY Auto-negotiation to support 10BaseT Half duplex,
    // 10BaseT Full duplex, 100BaseTX Half Duplex, 100BaseTX Full Duplex,
    // and symmetric PAUSE capability
    enc424j600WritePHYReg(PHANA, PHANA_ADPAUS0 | PHANA_AD10FD | PHANA_AD10 | PHANA_AD100FD | PHANA_AD100 | PHANA_ADIEEE0);

    // Enable RX packet reception
    enc424j600BFSReg(ECON1, ECON1_RXEN);
}

/********************************************************************
 * PACKET TRANSMISSION
 * ******************************************************************/

/**
 * Recieves packet
 * */
u16 enc424j600PacketReceive(u16 len, u08* packet) {
    u16 newRXTail;
    RXSTATUS statusVector;

    if (!(enc424j600ReadReg(EIR) & EIR_PKTIF)) {
        return FALSE;
    }

#ifdef AUTO_ICMP_ECHO  //Check if packet is ICMP echo packet and answer to it automaticaly
    //Set buffer for packet data
    u08 packetData[2];
    // Set the RX Read Pointer to the beginning of the next unprocessed packet + statusVektor + nextPacketPointer + position where paket type is saved
    enc424j600WriteReg(ERXRDPT, nextPacketPointer + sizeof (statusVector) + ETH_HEADER);

    //Read type of paket first, if it's IP
    enc424j600ReadMemoryWindow(RX_WINDOW, packetData, sizeof (packetData));
    if (packetData[0] == IP_PROTOCOL1 && packetData[1] == IP_PROTOCOL2) {
        //Ok, it's ip packet, check if it's icmp packet
        enc424j600WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + IP_PROTOCOL_POS);
        enc424j600ReadMemoryWindow(RX_WINDOW, packetData, 1);
        if (packetData[0] == ICMP_PROTOCOL) {
            //It's icmp packet, read lenght and do DMA copy operation from recieve buffer to transmit buffer
            enc424j600WriteReg(ERXRDPT, nextPacketPointer + 2);
            enc424j600ReadMemoryWindow(RX_WINDOW, packetData, 2);
            if (*(u16*) packetData < 1522) {
                //Now do DMA copy, first read length from IP packet
                u16 ipPacketLen;
                u08 ipHeaderLen;
                enc424j600WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + 14);
                enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & ipHeaderLen, 1);
                ipHeaderLen = (ipHeaderLen & 15)*4;
                enc424j600WriteReg(ERXRDPT, nextPacketPointer + 2 + sizeof (statusVector) + 16);
                enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & ipPacketLen, 2);
                ipPacketLen = HTONS(ipPacketLen);
                //Wait until controler is ready
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Set DMA copy and no checksum while copying (checksum computing at the end will be faster)
                //Switch MAC addr
                enc424j600BFSReg(ECON1, ECON1_DMACPY);
                enc424j600BFSReg(ECON1, ECON1_DMANOCS);
                enc424j600WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 6); //Switch MAC addr in packet
                enc424j600WriteReg(EDMADST, TXSTART);
                enc424j600WriteReg(EDMALEN, 6);
                enc424j600BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                enc424j600WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector)); //Switch MAC addr in packet
                enc424j600WriteReg(EDMADST, TXSTART + 6);
                enc424j600WriteReg(EDMALEN, 6);
                enc424j600BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                enc424j600WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 12); //Copy packet
                enc424j600WriteReg(EDMADST, TXSTART + 12);
                enc424j600WriteReg(EDMALEN, ipPacketLen + 2);
                enc424j600BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Switch IP addr
                enc424j600WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 26); //Switch IP addr in packet
                enc424j600WriteReg(EDMADST, TXSTART + 30);
                enc424j600WriteReg(EDMALEN, 4);
                enc424j600BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                enc424j600WriteReg(EDMAST, nextPacketPointer + 2 + sizeof (statusVector) + 30); //Switch IP addr in packet
                enc424j600WriteReg(EDMADST, TXSTART + 26);
                enc424j600WriteReg(EDMALEN, 4);
                enc424j600BFSReg(ECON1, ECON1_DMAST); // Wait until done
                while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
                }
                //Change echo request to echo reply
                packetData[0] = 0;
                packetData[1] = 0;
                enc424j600WriteReg(EGPWRPT, 34);
                enc424j600WriteMemoryWindow(GP_WINDOW, packetData, 1);
                enc424j600WriteReg(EGPWRPT, 36);
                enc424j600WriteMemoryWindow(GP_WINDOW, packetData, 2);
                //Compute checksum (use packetData for mem saving)
                *(u16*) packetData = enc424j600ChecksumCalculation(ETH_HEADER + ipHeaderLen, ipPacketLen - ipHeaderLen, 0x0000);
                //Write it to the packet
                enc424j600WriteReg(EGPWRPT, 36);
                enc424j600WriteMemoryWindow(GP_WINDOW, packetData, 2);
                //Flush packet out
                enc424j600WriteReg(ETXLEN, ipPacketLen + ETH_HEADER);
                enc424j600MACFlush();
            }
            enc424j600WriteReg(ERXRDPT, nextPacketPointer);
            enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & nextPacketPointer, sizeof (nextPacketPointer));
            newRXTail = nextPacketPointer - 2;
            //Special situation if nextPacketPointer is exactly RXSTART
            if (nextPacketPointer == RXSTART)
                newRXTail = RAMSIZE - 2;
            //Packet decrement
            enc424j600BFSReg(ECON1, ECON1_PKTDEC);
            //Write new RX tail
            enc424j600WriteReg(ERXTAIL, newRXTail);
            //
            return 0;
        }
    } 
#endif

    // Set the RX Read Pointer to the beginning of the next unprocessed packet
    enc424j600WriteReg(ERXRDPT, nextPacketPointer);

    enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & nextPacketPointer, sizeof (nextPacketPointer));

    enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & statusVector, sizeof (statusVector));
    len = (statusVector.bits.ByteCount <= len+4) ? statusVector.bits.ByteCount-4 : 0;
    enc424j600ReadMemoryWindow(RX_WINDOW, packet, len);

    newRXTail = nextPacketPointer - 2;
    //Special situation if nextPacketPointer is exactly RXSTART
    if (nextPacketPointer == RXSTART)
        newRXTail = RAMSIZE - 2;

    //Packet decrement
    enc424j600BFSReg(ECON1, ECON1_PKTDEC);

    //Write new RX tail
    enc424j600WriteReg(ERXTAIL, newRXTail);

    return len;
}

/**
 * Sends packet
 * */
#include "LCD_driver.h"

void enc424j600PacketSend(u16 len, u08* packet) {
    // Set the Window Write Pointer to the beginning of the transmit buffer
    enc424j600WriteReg(EGPWRPT, TXSTART);

#ifdef HARDWARE_CHECKSUM_NULL
    // Is it the IP packet? If so, for sure null checksum a let hardware to compute it
    if (packet[12] == IP_PROTOCOL1 && packet[13] == IP_PROTOCOL2) {
        //clear IP checksum
        packet[24] = 0;
        packet[25] = 0;
        //we can also compute icmp/tcp/udp messages
        if (packet[IP_PROTOCOL_POS] == ICMP_PROTOCOL) {
            //clear ICMP checksum
            packet[36] = 0;
            packet[37] = 0;
        } else if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL) {
            //clear TCP checksum
            packet[50] = 0;
            packet[51] = 0;
        } else if (packet[IP_PROTOCOL_POS] == UDP_PROTOCOL) {
            //clear UDP checksum
            packet[40] = 0;
            packet[41] = 0;
        }
    }
#endif

    enc424j600WriteMemoryWindow(GP_WINDOW, packet, len);

#ifdef HARDWARE_CHECKSUM
    // Is it the IP packet? Get it computed by hardware
    if (packet[12] == IP_PROTOCOL1 && packet[13] == IP_PROTOCOL2) {
        //Compute header length
        u08 headerLen = (packet[ETH_HEADER] & 15)*4;
        //Compute checksum of IP header
        u16 checksum = enc424j600ChecksumCalculation(ETH_HEADER, headerLen, 0x0000);
        //Write it to correct position
        enc424j600WriteReg(EGPWRPT, 24);
        enc424j600WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);

        //we can also compute icmp/tcp/udp messages
        if (packet[IP_PROTOCOL_POS] == ICMP_PROTOCOL) { /*ICMP*/
            //Compute header length
            u08 icmpLen = len - headerLen - ETH_HEADER;
            //Compute checksum of ICMP
            checksum = enc424j600ChecksumCalculation(ETH_HEADER + headerLen, icmpLen, 0x0000);
            //Write it to correct position
            enc424j600WriteReg(EGPWRPT, 36);
            enc424j600WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);
        } else if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL || packet[IP_PROTOCOL_POS] == UDP_PROTOCOL) { /*TCP or UDP*/
            //Compute header length
            u16 upperLayerLen = len - headerLen - ETH_HEADER;

            //Compute checksum of TCP or UDP
            checksum = ~(HTONS(packet[IP_PROTOCOL_POS] + upperLayerLen)); //HTONS macro is from uIP
            checksum = enc424j600ChecksumCalculation(26, 8, checksum);
            checksum = enc424j600ChecksumCalculation(ETH_HEADER + headerLen, upperLayerLen, checksum);

            //Write it to correct position
            if (packet[IP_PROTOCOL_POS] == TCP_PROTOCOL) {
                enc424j600WriteReg(EGPWRPT, 50);
            } else {
                enc424j600WriteReg(EGPWRPT, 40);
            }
            enc424j600WriteMemoryWindow(GP_WINDOW, ((u08*) & checksum), 2);
        }
    }
#endif

    enc424j600WriteReg(ETXLEN, len);

    enc424j600MACFlush();


}

/**
 * Reads MAC address of device
 * */
void enc424j600ReadMacAddr(u08 * macAddr) {
    // Get MAC adress
    u16 regValue;
    regValue = enc424j600ReadReg(MAADR1);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];
    regValue = enc424j600ReadReg(MAADR2);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];
    regValue = enc424j600ReadReg(MAADR3);
    *macAddr++ = ((u08*) & regValue)[0];
    *macAddr++ = ((u08*) & regValue)[1];
}

/**
 * Sets MAC address of device
 * */
void enc424j600SetMacAddr(u08 * macAddr) {
    u16 regValue;
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    enc424j600WriteReg(MAADR1, regValue);
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    enc424j600WriteReg(MAADR2, regValue);
    ((u08*) & regValue)[0] = *macAddr++;
    ((u08*) & regValue)[1] = *macAddr++;
    enc424j600WriteReg(MAADR3, regValue);
}

/**
 * Enables powersave mode
 * */
void enc424j600PowerSaveEnable(void) {
    //Turn off modular exponentiation and AES engine
    enc424j600BFCReg(EIR, EIR_CRYPTEN);
    //Turn off packet reception
    enc424j600BFCReg(ECON1, ECON1_RXEN);
    //Wait for any in-progress receptions to complete
    while (enc424j600ReadReg(ESTAT) & ESTAT_RXBUSY) {
        _delay_us(100);
    }
    //Wait for any current transmisions to complete
    while (enc424j600ReadReg(ECON1) & ECON1_TXRTS) {
        _delay_us(100);
    }
    //Power-down PHY
    u16 state;
    state = enc424j600ReadPHYReg(PHCON1);
    enc424j600WritePHYReg(PHCON1, state | PHCON1_PSLEEP);
    //Power-down eth interface
    enc424j600BFCReg(ECON2, ECON2_ETHEN);
    enc424j600BFCReg(ECON2, ECON2_STRCH);
}

/**
 * Disables powersave mode
 * */
void enc424j600PowerSaveDisable(void) {
    //Wake-up eth interface
    enc424j600BFSReg(ECON2, ECON2_ETHEN);
    enc424j600BFSReg(ECON2, ECON2_STRCH);
    //Wake-up PHY
    u16 state;
    state = enc424j600ReadPHYReg(PHCON1);
    enc424j600WritePHYReg(PHCON1, state & ~PHCON1_PSLEEP);
    //Turn on packet reception
    enc424j600BFSReg(ECON1, ECON1_RXEN);
}

/**
 * Is link connected?
 * @return <bool>
 */
bool enc424j600IsLinked(void) {
    return (enc424j600ReadReg(ESTAT) & ESTAT_PHYLNK) != 0u;
}

/**
 * Saves data to userspace defined by USSTART & USEND
 * @return bool (true if saved, false if there is no space)
 * */
bool enc424j600SaveToUserSpace(u16 position, u08* data, u16 len) {
    if ((USSTART + position + len) > USEND) return false;
    enc424j600WriteReg(EUDAWRPT, USSTART + position);
    enc424j600WriteMemoryWindow(UDA_WINDOW, data, len);
    return true;
}

/**
 * Loads data from userspace defined by USSTART & USEND
 * @return bool (true if area is in userspace, false if asked area is out of userspace)
 * */
bool enc424j600ReadFromUserSpace(u16 position, u08* data, u16 len) {
    if ((USSTART + position + len) > USEND) return false;
    enc424j600WriteReg(EUDARDPT, USSTART + position);
    enc424j600ReadMemoryWindow(UDA_WINDOW, data, len);
    return true;
}

/********************************************************************
 * UTILS
 * ******************************************************************/

static void enc424j600SendSystemReset(void) {
    // Perform a reset via the SPI/PSP interface
    do {
        // Set and clear a few bits that clears themselves upon reset.
        // If EUDAST cannot be written to and your code gets stuck in this
        // loop, you have a hardware problem of some sort (SPI or PMP not
        // initialized correctly, I/O pins aren't connected or are
        // shorted to something, power isn't available, etc.)
        sbi(PORTE, PE7);
        do {
            enc424j600WriteReg(EUDAST, 0x1234);
        } while (enc424j600ReadReg(EUDAST) != 0x1234);
        // Issue a reset and wait for it to complete
        enc424j600BFSReg(ECON2, ECON2_ETHRST);
        currentBank = 0;
        while ((enc424j600ReadReg(ESTAT) & (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY)) != (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY));
        _delay_us(300);
        // Check to see if the reset operation was successful by
        // checking if EUDAST went back to its reset default.  This test
        // should always pass, but certain special conditions might make
        // this test fail, such as a PSP pin shorted to logic high.
    } while (enc424j600ReadReg(EUDAST) != 0x0000u);

    // Really ensure reset is done and give some time for power to be stable
    _delay_us(1000);
}

/**
 * Is transmission active?
 * @return <bool>
 */
static bool enc424j600MACIsTxReady(void) {
    return !(enc424j600ReadReg(ECON1) & ECON1_TXRTS);
}

static void enc424j600MACFlush(void) {
    // Check to see if the duplex status has changed.  This can
    // change if the user unplugs the cable and plugs it into a
    // different node.  Auto-negotiation will automatically set
    // the duplex in the PHY, but we must also update the MAC
    // inter-packet gap timing and duplex state to match.
    if (enc424j600ReadReg(EIR) & EIR_LINKIF) {
        enc424j600BFCReg(EIR, EIR_LINKIF);

        u16 w;

        // Update MAC duplex settings to match PHY duplex setting
        w = enc424j600ReadReg(MACON2);
        if (enc424j600ReadReg(ESTAT) & ESTAT_PHYDPX) {
            // Switching to full duplex
            enc424j600WriteReg(MABBIPG, 0x15);
            w |= MACON2_FULDPX;
        } else {
            // Switching to half duplex
            enc424j600WriteReg(MABBIPG, 0x12);
            w &= ~MACON2_FULDPX;
        }
        enc424j600WriteReg(MACON2, w);
    }


    // Start the transmission, but only if we are linked.
    if (enc424j600ReadReg(ESTAT) & ESTAT_PHYLNK)
        enc424j600BFSReg(ECON1, ECON1_TXRTS);
}

/**
 * Calculates IP checksum value
 *
 * */
static u16 enc424j600ChecksumCalculation(u16 position, u16 length, u16 seed) {
    // Wait until module is idle
    while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
    }
    // Clear DMACPY to prevent a copy operation
    enc424j600BFCReg(ECON1, ECON1_DMACPY);
    // Clear DMANOCS to select a checksum operation
    enc424j600BFCReg(ECON1, ECON1_DMANOCS);
    // Clear DMACSSD to use the default seed of 0000h
    enc424j600BFCReg(ECON1, ECON1_DMACSSD);
    // Set EDMAST to source address
    enc424j600WriteReg(EDMAST, position);
    // Set EDMALEN to length
    enc424j600WriteReg(EDMALEN, length);
    //If we have a seed, now it's time
    if (seed) {
        enc424j600BFSReg(ECON1, ECON1_DMACSSD);
        enc424j600WriteReg(EDMACS, seed);
    }
    // Initiate operation
    enc424j600BFSReg(ECON1, ECON1_DMAST);
    // Wait until done
    while (enc424j600ReadReg(ECON1) & ECON1_DMAST) {
    }
    return enc424j600ReadReg(EDMACS);
}

/********************************************************************
 * READERS AND WRITERS
 * ******************************************************************/

static void enc424j600WriteMemoryWindow(u08 window, u08 *data, u16 length) {
    u08 op = WBMUDA;

    if (window & GP_WINDOW)
        op = WBMGP;
    if (window & RX_WINDOW)
        op = WBMRX;

    enc424j600WriteN(op, data, length);
}

static void enc424j600ReadMemoryWindow(u08 window, u08 *data, u16 length) {
    if (length == 0u)
        return;

    u08 op = RBMUDA;

    if (window & GP_WINDOW)
        op = RBMGP;
    if (window & RX_WINDOW)
        op = RBMRX;

    enc424j600ReadN(op, data, length);
}

/**
 * Reads from address
 * @variable <u16> address - register address
 * @return <u16> data - data in register
 */
static u16 enc424j600ReadReg(u16 address) {
    u16 returnValue;
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    //If address is banked, we will use banked access
    if (bank <= (0x3u << 5)) {
        if (bank != currentBank) {
            if (bank == (0x0u << 5))
                enc424j600ExecuteOp0(B0SEL);
            else if (bank == (0x1u << 5))
                enc424j600ExecuteOp0(B1SEL);
            else if (bank == (0x2u << 5))
                enc424j600ExecuteOp0(B2SEL);
            else if (bank == (0x3u << 5))
                enc424j600ExecuteOp0(B3SEL);

            currentBank = bank;
        }
        returnValue = enc424j600ExecuteOp16(RCR | (address & 0x1F), 0x0000);
    } else {
        u32 returnValue32 = enc424j600ExecuteOp32(RCRU, (u32) address);
        ((u08*) & returnValue)[0] = ((u08*) & returnValue32)[1];
        ((u08*) & returnValue)[1] = ((u08*) & returnValue32)[2];
    }

    return returnValue;
}

/**
 * Writes to register
 * @variable <u16> address - register address
 * @variable <u16> data - data to register
 */
static void enc424j600WriteReg(u16 address, u16 data) {
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    //If address is banked, we will use banked access
    if (bank <= (0x3u << 5)) {
        if (bank != currentBank) {
            if (bank == (0x0u << 5))
                enc424j600ExecuteOp0(B0SEL);
            else if (bank == (0x1u << 5))
                enc424j600ExecuteOp0(B1SEL);
            else if (bank == (0x2u << 5))
                enc424j600ExecuteOp0(B2SEL);
            else if (bank == (0x3u << 5))
                enc424j600ExecuteOp0(B3SEL);

            currentBank = bank;
        }
        enc424j600ExecuteOp16(WCR | (address & 0x1F), data);
    } else {
        u32 data32;
        ((u08*) & data32)[0] = (u08) address;
        ((u08*) & data32)[1] = ((u08*) & data)[0];
        ((u08*) & data32)[2] = ((u08*) & data)[1];
        enc424j600ExecuteOp32(WCRU, data32);
    }

}

static u16 enc424j600ReadPHYReg(u08 address) {
    u16 returnValue;

    // Set the right address and start the register read operation
    enc424j600WriteReg(MIREGADR, 0x0100 | address);
    enc424j600WriteReg(MICMD, MICMD_MIIRD);

    // Loop to wait until the PHY register has been read through the MII
    // This requires 25.6us
    while (enc424j600ReadReg(MISTAT) & MISTAT_BUSY);

    // Stop reading
    enc424j600WriteReg(MICMD, 0x0000);

    // Obtain results and return
    returnValue = enc424j600ReadReg(MIRD);

    return returnValue;
}

static void enc424j600WritePHYReg(u08 address, u16 Data) {
    // Write the register address
    enc424j600WriteReg(MIREGADR, 0x0100 | address);

    // Write the data
    enc424j600WriteReg(MIWR, Data);

    // Wait until the PHY register has been written
    while (enc424j600ReadReg(MISTAT) & MISTAT_BUSY);
}

static void enc424j600ReadN(u08 op, u08* data, u16 dataLen) {
    // assert CS
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue read command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));

    while (dataLen--) {
        // wait for answer
        SPDR = 0x00;
        while (!(inb(SPSR) & (1 << SPIF)));
        *data++ = SPDR;
    }

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);
}

static void enc424j600WriteN(u08 op, u08* data, u16 dataLen) {
    // assert CS
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue read command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));

    while (dataLen--) {
        // start sending data to SPI
        SPDR = *data++;
        // wail until all is sent
        while (!(SPSR & (1 << SPIF)));
    }

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);
}

static void enc424j600BFSReg(u16 address, u16 bitMask) {
    u08 bank;

    // See if we need to change register banks
    bank = ((BYTE) address) & 0xE0;
    if (bank != currentBank) {
        if (bank == (0x0u << 5))
            enc424j600ExecuteOp0(B0SEL);
        else if (bank == (0x1u << 5))
            enc424j600ExecuteOp0(B1SEL);
        else if (bank == (0x2u << 5))
            enc424j600ExecuteOp0(B2SEL);
        else if (bank == (0x3u << 5))
            enc424j600ExecuteOp0(B3SEL);

        currentBank = bank;
    }

    enc424j600ExecuteOp16(BFS | (address & 0x1F), bitMask);
}

static void enc424j600BFCReg(u16 address, u16 bitMask) {
    u08 bank;

    // See if we need to change register banks
    bank = ((u08) address) & 0xE0;
    if (bank != currentBank) {
        if (bank == (0x0u << 5))
            enc424j600ExecuteOp0(B0SEL);
        else if (bank == (0x1u << 5))
            enc424j600ExecuteOp0(B1SEL);
        else if (bank == (0x2u << 5))
            enc424j600ExecuteOp0(B2SEL);
        else if (bank == (0x3u << 5))
            enc424j600ExecuteOp0(B3SEL);

        currentBank = bank;
    }

    enc424j600ExecuteOp16(BFC | (address & 0x1F), bitMask);
}
/********************************************************************
 * EXECUTES
 * ******************************************************************/

/**
 * Execute SPI operation
 * @variable <u08> op - operation
 */
static void enc424j600ExecuteOp0(u08 op) {
    u08 dummy;
    // assert CS
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue read command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // read answer
    dummy = SPDR;

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u08> data - data
 */
static u08 enc424j600ExecuteOp8(u08 op, u08 data) {
    u08 returnValue;
    // assert CS
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // start sending data to SPI
    SPDR = data;
    // wain until all is sent
    while (!(SPSR & (1 << SPIF)));
    // read answer
    returnValue = SPDR;
    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);

    return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u16> data - data
 */
static u16 enc424j600ExecuteOp16(u08 op, u16 data) {
    u16 returnValue;
    // assert CS
    // zabereme sbernici
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // in this cycle, data are sent
    for (int x = 0; x < 2; x++) {
        // start sending data to SPI
        SPDR = ((u08*) & data)[x];
        // wain until all is sent
        while (!(SPSR & (1 << SPIF)));
        // read answer
        ((u08*) & returnValue)[x] = SPDR;
    }

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);

    return returnValue;
}

/**
 * Write data to SPI with operation
 * @variable <u08> op - SPI operation
 * @variable <u32> data - data
 */
static u32 enc424j600ExecuteOp32(u08 op, u32 data) {
    u16 returnValue;
    // assert CS
    ENC424J600_CONTROL_PORT &= ~(1 << ENC424J600_CONTROL_CS);

    // issue write command
    SPDR = op;
    // wail until all is sent
    while (!(SPSR & (1 << SPIF)));
    // in this cycle, data are sent
    for (int x = 0; x < 3; x++) {
        // start sending data to SPI
        SPDR = ((u08*) & data)[x];
        // wain until all is sent
        while (!(SPSR & (1 << SPIF)));
        // read answer
        ((u08*) & returnValue)[x] = SPDR;
    }

    // release CS
    ENC424J600_CONTROL_PORT |= (1 << ENC424J600_CONTROL_CS);

    return returnValue;
}