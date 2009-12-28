/******************************************
 * Title        : Microchip ENCX24J600 Ethernet Interface Driver
 * Author       : Jiri Melnikov
 * Created      : 28.12.2009
 * Version      : 0.1
 * Target MCU   : Atmel AVR series
 *
 * Description  : This driver provides initialization and transmit/receive
 *                functions for the Microchip ENCX24J600 100Mb Ethernet
 *                Controller and PHY. Only supported interface is SPI, no
 *                PSP interface available by now. No security functions are
 *                are supported by now.
 *
 *                This driver is inspired by ENC28J60 driver from Pascal
 *                Stang (2005).
 *
 *                Many lines of code are taken from Microchip's TCP/IP stack.
 * 
 * ****************************************/

#include "enc424j600.h"
#include "enc424j600conf.h"
#include "avrlibdefs.h"
#include "LCD_driver.h"
#include <avr/iom169.h>
#include <avr/io.h>
#include <util/delay.h>

// Binary constant identifiers for ReadMemoryWindow() and WriteMemoryWindow()
// functions
#define UDA_WINDOW		(0x1)
#define GP_WINDOW		(0x2)
#define RX_WINDOW		(0x4)

// Promiscuous mode, uncomment if you want to receive all packets, even those which are not for you
// #define PROMISCUOUS_MODE

// Internal MAC level variables and flags.
static u08 currentBank;
static u16 nextPacketPointer;

// External MAC an IP address variable
extern MAC_ADDR mac_addr;
extern IP_ADDR ip_addr;


void enc424j600Init(void);
u16 enc424j600PacketReceive(u16 maxlen, u08* packet);
void enc424j600PacketSend(u16 len, u08* packet);

void enc424j600MACFlush(void);
static void enc424j600SendSystemReset(void);
static u16 enc424j600ReadReg(u16 address);
static void enc424j600WriteReg(u16 address, u16 data);
u16 enc424j600ReadPHYReg(u08 address);
void enc424j600WritePHYReg(u08 address, u16 Data);
static void enc424j600ExecuteOp0(u08 op);
u08 enc424j600ExecuteOp8(u08 op, u08 data);
u16 enc424j600ExecuteOp16(u08 op, u16 data);
u32 enc424j600ExecuteOp32(u08 op, u32 data);
static void enc424j600BFSReg(u16 address, u16 bitMask);
static void enc424j600BFCReg(u16 address, u16 bitMask);
void enc424j600ReadMemoryWindow(u08 window, u08 *data, u16 length);
void enc424j600WriteMemoryWindow(u08 window, u08 *data, u16 length);
static void enc424j600WriteN(u08 op, u08* data, u16 dataLen);
static void enc424j600ReadN(u08 op, u08* data, u16 dataLen);

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
    enc424j600WriteReg(EUDAST, RAMSIZE);
    enc424j600WriteReg(EUDAND, RAMSIZE + 1);

    // Get MAC adress
    u16 regValue;
    regValue = enc424j600ReadReg(MAADR1);
    mac_addr.v[0] = ((u08*) & regValue)[0];
    mac_addr.v[1] = ((u08*) & regValue)[1];
    regValue = enc424j600ReadReg(MAADR2);
    mac_addr.v[2] = ((u08*) & regValue)[0];
    mac_addr.v[3] = ((u08*) & regValue)[1];
    regValue = enc424j600ReadReg(MAADR3);
    mac_addr.v[4] = ((u08*) & regValue)[0];
    mac_addr.v[5] = ((u08*) & regValue)[1];

    // If promiscuous mode is set, than allow accept all packets
    #ifdef PROMISCUOUS_MODE
    enc424j600WriteReg(ERXFCON,(ERXFCON_CRCEN | ERXFCON_RUNTEN | ERXFCON_UCEN | ERXFCON_NOTMEEN | ERXFCON_MCEN));
    #endif

    // Set PHY Auto-negotiation to support 10BaseT Half duplex,
    // 10BaseT Full duplex, 100BaseTX Half Duplex, 100BaseTX Full Duplex,
    // and symmetric PAUSE capability
    enc424j600WritePHYReg(PHANA, PHANA_ADPAUS0 | PHANA_AD10FD | PHANA_AD10 | PHANA_AD100FD | PHANA_AD100 | PHANA_ADIEEE0);

    // Enable RX packet reception
    enc424j600BFSReg(ECON1, ECON1_RXEN);
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
        currentBank = 0; while ((enc424j600ReadReg(ESTAT) & (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY)) != (ESTAT_CLKRDY | ESTAT_RSTDONE | ESTAT_PHYRDY));
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
 * Is link connected?
 * @return <bool>
 */
BOOL enc424j600MACIsLinked(void) {
    return (enc424j600ReadReg(ESTAT) & ESTAT_PHYLNK) != 0u;
}

/**
 * Is transmission active?
 * @return <bool>
 */
BOOL enc424j600MACIsTxReady(void) {
    return !(enc424j600ReadReg(ECON1) & ECON1_TXRTS);
}

/********************************************************************
 * PACKET TRANSMISSION
 * ******************************************************************/

u16 enc424j600PacketReceive(u16 len, u08* packet) {
    u16 newRXTail;
    RXSTATUS statusVector;

    if (!(enc424j600ReadReg(EIR) & EIR_PKTIF)) {
        return FALSE;
    }


    // Set the RX Read Pointer to the beginning of the next unprocessed packet
    enc424j600WriteReg(ERXRDPT, nextPacketPointer);

    
    enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & nextPacketPointer, sizeof (nextPacketPointer));
    enc424j600ReadMemoryWindow(RX_WINDOW, (u08*) & statusVector, sizeof (statusVector));
    if (statusVector.bits.ByteCount <= len) len = statusVector.bits.ByteCount;
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

void enc424j600PacketSend(u16 len, u08* packet) {
    // Set the Window Write Pointer to the beginning of the transmit buffer
    enc424j600WriteMemoryWindow(GP_WINDOW, packet, len);

    enc424j600WriteReg(EGPWRPT, TXSTART);
    enc424j600WriteReg(ETXLEN, len);

    enc424j600MACFlush();


}

void enc424j600MACFlush(void) {
    u16 w;

    // Check to see if the duplex status has changed.  This can
    // change if the user unplugs the cable and plugs it into a
    // different node.  Auto-negotiation will automatically set
    // the duplex in the PHY, but we must also update the MAC
    // inter-packet gap timing and duplex state to match.
    if (enc424j600ReadReg(EIR) & EIR_LINKIF) {
        enc424j600BFCReg(EIR, EIR_LINKIF);

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


    // Start the transmission, but only if we are linked.  Supressing
    // transmissing when unlinked is necessary to avoid stalling the TX engine
    // if we are in PHY energy detect power down mode and no link is present.
    // A stalled TX engine won't do any harm in itself, but will cause the
    // MACIsTXReady() function to continuously return FALSE, which will
    // ultimately stall the Microchip TCP/IP stack since there is blocking code
    // elsewhere in other files that expect the TX engine to always self-free
    // itself very quickly.
    if (enc424j600ReadReg(ESTAT) & ESTAT_PHYLNK)
        enc424j600BFSReg(ECON1, ECON1_TXRTS);
}

/********************************************************************
 * READERS AND WRITERS
 * ******************************************************************/

void enc424j600WriteMemoryWindow(u08 window, u08 *data, u16 length) {
    u08 op = RBMUDA;

    if (window & GP_WINDOW)
        op = WBMGP;
    if (window & RX_WINDOW)
        op = WBMRX;

    enc424j600WriteN(op, data, length);
}

void enc424j600ReadMemoryWindow(u08 window, u08 *data, u16 length) {
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

u16 enc424j600ReadPHYReg(u08 address) {
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

void enc424j600WritePHYReg(u08 address, u16 Data) {
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
u08 enc424j600ExecuteOp8(u08 op, u08 data) {
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
u16 enc424j600ExecuteOp16(u08 op, u16 data) {
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
u32 enc424j600ExecuteOp32(u08 op, u32 data) {
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