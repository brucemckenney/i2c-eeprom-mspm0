///
//  i2c-eeprom.c
//  SLAA208 updated for the MSPM0
//  Copyright Bruce McKenney 2025
//  BSD 2-clause license
//

//  We don't include "ti_msp_dl_config.h" since main() takes care of all that
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "i2c-eeprom.h"

#define ETRACE  0       // I'm still suspicious of some of the MSR bits (but not today)
#define ERRCNT  1
#define RS_WA   1       // Workaround for RD_ON_TXEMPTY (driverlib) hazard

///
//  eep_param block
//  Saved here so they don't need to be provided to each function.
//  This preserves the signatures from SLAA208.
//
typedef struct _eep_params
{
    I2C_Regs *ep_i2c;
    uint8_t ep_i2c_addr;    // Target (EEPROM) I2C address
#if EEP_DMA
    uint8_t ep_dma_chanid;
#endif // EEP_DMA
}eep_params;
eep_params eep_param;
eep_params * const eep = &eep_param;

#if ERRCNT
uint32_t eep_errcnt;
#endif // ERRCNT
uint32_t eep_idlewait;          // Monitor how long it takes to go IDLE

///
//  InitI2C()
//  i2cdev should point to a configured I2C unit, e.g. from sysconfig.
//  eeprom_i2c_address should be 7-bits (right-justified).
//
void
InitI2C(I2C_Regs *i2cdev, unsigned char eeprom_i2c_address)
{
    eep->ep_i2c = i2cdev;
    eep->ep_i2c_addr = eeprom_i2c_address;
#if EEP_DMA
    eep->ep_dma_chanid = EEP_DMA_NOCHAN;
#endif // EEP_DMA

    return;
}

#if EEP_DMA
void
InitI2C_DMA(I2C_Regs *i2c, unsigned char eeprom_i2c_address, uint8_t chanid)
{
    eep->ep_i2c = i2c;
    eep->ep_i2c_addr = eeprom_i2c_address;
    eep->ep_dma_chanid = chanid;
    if (chanid != EEP_DMA_NOCHAN)
    {
        // Clear out whatever Sysconfig set
        DL_I2C_disableDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1,    // DMA_TRIG1
                           (DL_I2C_DMA_INTERRUPT_TARGET_RXFIFO_TRIGGER|DL_I2C_DMA_INTERRUPT_TARGET_RXFIFO_TRIGGER));
    }

    return;
}
#endif // EEP_DMA

///
//  EEPROM_WaitIdle()
//  Wait for IDLE status. This may come a "long" time after apparent completion.
//  This should be used after completion is indicated/before starting a new operation.
//
static inline void
EEPROM_WaitIdle(I2C_Regs *i2c)
{
    while ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0)
    {
        ++eep_idlewait;
    }
    return;
}

///
//  EEPROM_SetAddr()
//  Shorthand: Stuff the EEPROM memory address into the Tx FIFO as a prefix
//
static inline void
EEPROM_SetAddr(I2C_Regs *i2c, unsigned int addr)
{
    if (sizeof(eep_addr) > 3)
        DL_I2C_transmitControllerData(i2c, (addr >> 24) & 0xFF);
    if (sizeof(eep_addr) > 2)
        DL_I2C_transmitControllerData(i2c, (addr >> 16) & 0xFF);
    if (sizeof(eep_addr) > 1)
        DL_I2C_transmitControllerData(i2c, (addr >>  8) & 0xFF);

    DL_I2C_transmitControllerData(i2c, (addr >> 0) & 0xFF);

    return;
}

///
//  EEPROM_drainRxFIFO()
//  Useful shorthand to unload the Rx FIFO.
//
static inline unsigned
EEPROM_drainRxFIFO(I2C_Regs *i2c, unsigned i, unsigned char *Data, unsigned Size)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(i2c))
    {
        uint8_t c;
        c = DL_I2C_receiveControllerData(i2c);
        if (i < Size)                // Safety first
        {
            Data[i] = c;
            ++i;
        }
    } // while (something in the Rx FIFO)

    return(i);
}

///
//  EEPROM_ByteWrite()
//
void
EEPROM_ByteWrite(unsigned int Address , unsigned char Data)
{
    //  A Byte Write is just a 1-byte Page Write
    unsigned char dat[1];

    dat[0] = Data;
    EEPROM_PageWrite(Address, &dat[0], 1);

    return;
}

///
//  EEPROM_PageWrite()
//
void
EEPROM_PageWrite(unsigned int Address , unsigned char * Data , unsigned int Size)
{
    I2C_Regs *i2c = eep->ep_i2c;
#if EEP_DMA
    uint8_t dmachan = eep->ep_dma_chanid;
#endif // EEP_DMA

    unsigned cnt = Size;
    unsigned char *ptr = Data;
    eep_addr addr = (eep_addr)Address;  // Wrap address as needed

#if EEP_DMA
    if (dmachan != EEP_DMA_NOCHAN)
    {
        DL_DMA_setDestAddr(DMA, dmachan, (uint32_t)&i2c->MASTER.MTXDATA);
        DL_I2C_enableDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER); // DMA_TRIG1
        DL_DMA_configTransfer(DMA, dmachan,
                              DL_DMA_SINGLE_TRANSFER_MODE, DL_DMA_NORMAL_MODE,
                              DL_DMA_WIDTH_BYTE, DL_DMA_WIDTH_BYTE,
                              DL_DMA_ADDR_INCREMENT, DL_DMA_ADDR_UNCHANGED);// inc src, not dest
   }
#endif // EEP_DMA
    //  Fill pages until we run out of data.
    while (cnt > 0)
    {
        unsigned fragsiz, fragcnt;
        eep_addr pagetop;

        //  See how much can fit into the requested page
        pagetop = (addr + EEP_PAGESIZE) & ~(EEP_PAGESIZE-1); // addr of next page
        fragsiz = pagetop - addr;       // Amount that can fit in this page
        if (fragsiz > cnt)              // Don't go overboard
            fragsiz = cnt;
        fragcnt = fragsiz;              // Prepare to count

        EEPROM_WaitIdle(i2c);

        //  Stuff the EEPROM memory address into the Tx FIFO as a prefix
        EEPROM_SetAddr(i2c, addr);

        //  Start the write for this fragment
        DL_I2C_clearInterruptStatus(i2c, (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE|DL_I2C_INTERRUPT_CONTROLLER_NACK));
        DL_I2C_startControllerTransfer(i2c, eep->ep_i2c_addr,
            DL_I2C_CONTROLLER_DIRECTION_TX, sizeof(eep_addr)+fragsiz);  // Start
#if EEP_DMA
        if (dmachan != EEP_DMA_NOCHAN)
        {
            DL_DMA_setSrcAddr(DMA, dmachan, (uint32_t)ptr);
            DL_DMA_setTransferSize(DMA, dmachan, fragsiz); // don't count what's already in the FIFO
            DL_I2C_clearDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            DL_DMA_enableChannel(DMA, dmachan); // Prime
        }
#endif // EEP_DMA

        //  Busy-wait for completion
        do {
#if EEP_DMA
            if (dmachan != EEP_DMA_NOCHAN)
                continue;               // DMA is doing all the work
#endif // EEP_DMA
            // While waiting, try to keep the FIFO full.
            unsigned fillcnt;
            fillcnt = DL_I2C_fillControllerTXFIFO(i2c, ptr, fragcnt);
            fragcnt -= fillcnt;
            ptr += fillcnt;
        } while (!DL_I2C_getRawInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

#if ERRCNT
        if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR))
        { ++eep_errcnt; }
#endif // ERRCNT
        //  Move forward
        addr += fragsiz;
        cnt -= fragsiz;
#if EEP_DMA
        if (dmachan != EEP_DMA_NOCHAN)
        {
            ptr += fragsiz;
        }
#endif // EEP_DMA

        // Wait for EEPROM update to complete (Twr)
        EEPROM_AckPolling();
    } // while (cnt > 0)

    //  Only needed in case of an error
    DL_I2C_flushControllerTXFIFO(i2c);
#if EEP_DMA
    if (dmachan != EEP_DMA_NOCHAN)
    {
        DL_I2C_disableDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        DL_DMA_disableChannel(DMA, dmachan);
    }
#endif // EEP_DMA

    return;
}

///
//  EEPROM_RandomRead()
//
unsigned char
EEPROM_RandomRead(unsigned int Address)
{
    //  A RandomRead is just a 1-byte SequentialRead
    uint8_t dat[1];

    EEPROM_SequentialRead(Address, &dat[0], 1);

    return(dat[0]);
}

#if ETRACE
uint32_t eep_car_msr1, eep_car_msr2,eep_car_msr3;
#endif // ETRACE
///
//  EEPROM_CurrentAddressRead()
//
unsigned char
EEPROM_CurrentAddressRead(void)
{
    //  Read 1 byte without setting a new address.
    //  There's no reason to limit this to 1 byte, but that's what SLAA208 did.
    I2C_Regs *i2c = eep->ep_i2c;
    uint8_t dat[1];
    unsigned i;

    EEPROM_WaitIdle(i2c);

    //  Start the Rx transaction
    DL_I2C_clearInterruptStatus(i2c, (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE|DL_I2C_INTERRUPT_CONTROLLER_NACK));// Clear stale
    DL_I2C_startControllerTransfer(i2c, eep->ep_i2c_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, sizeof(dat));
#if ETRACE
    eep_car_msr1 = DL_I2C_getControllerStatus(i2c);
    delay_cycles(10*32);
    eep_car_msr2 = DL_I2C_getControllerStatus(i2c);
#endif // ETRACE

    //  Busy-wait for it to complete. A single byte fits in the FIFO.
    i = 0;
    do
    {
#if ETRACE
        eep_car_msr3 = DL_I2C_getControllerStatus(i2c);
        delay_cycles(1*32);
#else
        /*EMPTY*/;
#endif // ETRACE
    } while(!DL_I2C_getRawInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    //  Retrieve the result
    i = EEPROM_drainRxFIFO(i2c, i, dat, sizeof(dat));

    #if ERRCNT
    if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR))
    { ++eep_errcnt; }
#endif // ERRCNT

    return(dat[0]);
}

#if ETRACE
uint32_t eep_sr_msr1, eep_sr_msr2,eep_sr_msr3;
#endif // ETRACE
///
//  EEPROM_SequentialRead()
//
void
EEPROM_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size)
{
    I2C_Regs *i2c = eep->ep_i2c;
#if EEP_DMA
    uint8_t dmachan = eep->ep_dma_chanid;
#endif // EEP_DMA
    unsigned i;

    EEPROM_WaitIdle(i2c);

    // Insert the address in the Tx FIFO as a prefix
    EEPROM_SetAddr(i2c, Address);

    //   Set RD_ON_TXEMPTY so it sends the Tx FIFO data first
#if RS_WA       // Hazard workaround
    i2c->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;// Set repeat-start, clear others
#else // RS_WA
    DL_I2C_enableControllerReadOnTXEmpty(i2c);   // Write then read
#endif // RS_WA

    //   Start the transaction
    DL_I2C_clearInterruptStatus(i2c, (DL_I2C_INTERRUPT_CONTROLLER_RX_DONE|DL_I2C_INTERRUPT_CONTROLLER_NACK));// Clear stale
    DL_I2C_startControllerTransfer(i2c, eep->ep_i2c_addr,
                     DL_I2C_CONTROLLER_DIRECTION_RX, Size);   // Don't count the Tx FIFO contents
#if EEP_DMA
    if (dmachan != EEP_DMA_NOCHAN)
    {
        DL_DMA_setSrcAddr(DMA, dmachan, (uint32_t)&i2c->MASTER.MRXDATA);
        DL_DMA_setDestAddr(DMA, dmachan, (uint32_t)Data);
        DL_DMA_setTransferSize(DMA, dmachan, Size); // don't count what's in the Tx FIFO
        DL_DMA_configTransfer(DMA, dmachan,
                              DL_DMA_SINGLE_TRANSFER_MODE, DL_DMA_NORMAL_MODE,
                              DL_DMA_WIDTH_BYTE, DL_DMA_WIDTH_BYTE,
                              DL_DMA_ADDR_UNCHANGED,  DL_DMA_ADDR_INCREMENT);// inc dest, not src
        DL_I2C_clearDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
        DL_I2C_enableDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
        DL_DMA_enableChannel(DMA, dmachan); // Prime
    }
#endif // EEP_DMA
#if ETRACE
    eep_sr_msr1 = DL_I2C_getControllerStatus(i2c);
    delay_cycles(10*32);
    eep_sr_msr2 = DL_I2C_getControllerStatus(i2c);
#endif // ETRACE

    //   Busy-wait for completion
    i = 0;
    do {
#if ETRACE
        eep_sr_msr3 = DL_I2C_getControllerStatus(i2c);
        delay_cycles(1*32);
#endif // ETRACE
#if EEP_DMA
        if (dmachan != EEP_DMA_NOCHAN)
            continue;
#endif // EEP_DMA
        //   While waiting, drain the Rx FIFO as needed
        i = EEPROM_drainRxFIFO(i2c, i, Data, Size);
    } while(!DL_I2C_getRawInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    //   One more Rx FIFO check to avoid a race.
    i = EEPROM_drainRxFIFO(i2c, i, Data, Size);

#if ERRCNT
    if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR))
    { ++eep_errcnt; }
#endif // ERRCNT

    //   We're done with RD_ON_TXEMPTY
#if RS_WA       // Hazard workaround
    i2c->MASTER.MCTR = 0*I2C_MCTR_RD_ON_TXEMPTY_ENABLE; // Clear repeat-start, clear others
#else // RS_WA
    DL_I2C_disableControllerReadOnTXEmpty(i2c);
#endif // RS_WA

    //  Only needed in case of an error
    DL_I2C_flushControllerTXFIFO(i2c);
#if EEP_DMA
    if (dmachan != EEP_DMA_NOCHAN)
    {
        DL_I2C_disableDMAEvent(i2c, DL_I2C_EVENT_ROUTE_1, DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
        DL_DMA_disableChannel(DMA, dmachan);
    }
#endif // EEP_DMA

    return;
}

///
//  EEPROM_AckPolling()
//  The EEPROM doesn't respond (NACKs) while it's writing its memory
//  (Twr, typically <5ms), so just keep poking it until it ACKs.
//
void
EEPROM_AckPolling(void)
{
    I2C_Regs *i2c = eep->ep_i2c;
    unsigned OK = 0;
    do
    {
        EEPROM_WaitIdle(i2c);

        //  Send a 0-byte (Tx) request until it succeeds.
        //  I2C_ERR_01/02 put serious restrictions on an Rx-mode Quick command.
        DL_I2C_clearInterruptStatus(i2c, (DL_I2C_INTERRUPT_CONTROLLER_TX_DONE|DL_I2C_INTERRUPT_CONTROLLER_NACK));
        DL_I2C_startControllerTransfer(i2c, eep->ep_i2c_addr,
            DL_I2C_CONTROLLER_DIRECTION_TX, 0);                 // Start

        do {
            /*EMPTY*/;                                          // Busy-wait for completion
        } while (!DL_I2C_getRawInterruptStatus(i2c, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

        if ((DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0)
            {OK = 1;}                                           // No error -> OK
    } while (!OK);

    return;
}
