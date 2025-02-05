## Example Summary
This is an update of TI AppNote SLAA208 for the MSPM0 series. It provides a simple interface to read and write an I2C EEPROM, such as the AT24C256.

The function signatures are (almost) the same as those in SLAA208. The only visible difference from the AppNote is that InitI2C requires an additional argument: a (configured) I2C device (I2C_Regs *)
to communicate with the EEPROM. sysconfig can provide this; it will be named something similar to "I2C_1_INST".

No error checking is performed, since SLAA208 did not provide for this.

The code relies on DriverLib, but not sysconfig.

Polling  (busy-waiting) is used rather than interrupts. This is because only sysconfig knows the ISR name.

This was tested using a CAT24C512 and an AT24C256. Both of these use 2-byte addresses.
To use a smaller device, e.g. AT24C02 (1-byte address), change the definitions of eep_addr and EEP_PAGESIZE accordingly.

## DMA
If EEP_DMA=1, an alternate initialization InitI2C_DMA() may be called with a 3rd argument which is a DMA channel number.

This channel should be configured in Sysconfig to use the (appropriate) I2C trigger. 
Choose the DMA_I2Cn_TX_TRIG [sic] option, which actually refers to the DMA_TRIG1 publisher, not to anything about TX. 

The other DMA channel configuration is overwritten.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| GPIOA | PA0 | Open-Drain Output |
| GPIOA | PA15 | Standard Output |
| SYSCTL |  |  |
| I2C1 | PB3 | I2C Serial Data line (SDA) |
| I2C1 | PB2 | I2C Serial Clock line (SCL) |
| EVENT |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA0 | GPIOA | PA0 | J27_9 | <ul><li>PA0 is 5V tolerant open-drain so it requires pull-up<br><ul><li>`J19 1:2` Use 3.3V pull-up<br><li>`J19 2:3` Use 5V pull-up</ul><br><li>PA0 can be connected to LED1<br><ul><li>`J4 ON` Connect to LED1<br><li>`J4 OFF` Disconnect from LED1</ul></ul> |
| PA15 | GPIOA | PA15 | J3_30 | <ul><li>This pin can be used for testing purposes in boosterpack connector<ul><li>Pin can be reconfigured for general purpose as necessary</ul></ul> |
| PB3 | I2C1 | SDA | J1_10 | <ul><li>PB3 can be connected to an on-board pull-up resistor<br><ul><li>`R60` is not soldered by default<br><li>Solder `R60` to use on-board pull-up</ul></ul> |
| PB2 | I2C1 | SCL | J1_9 | <ul><li>PB2 can be connected to an on-board pull-up resistor<br><ul><li>`R59` is not soldered by default<br><li>Solder `R59` to use on-board pull-up</ul></ul> |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations

This example was developed using an MSPM0G3507.

The source code should operate unchanged on any of the G, L, or C series; sysconfig may be used to re-target.

## Example Usage

Connect SDA and SCL between I2C Controller and the EEPROM. /WP should be tied High.
Note that I2C requires pull-up resistors. Internal pull-ups can be enabled in
SysConfig (see datasheet for resistance specification), but external pull-ups
might be required based on I2C speed and capacitance. External pull-ups can be
connected or enabled using the LaunchPad.

## Credits
The .syscfg and parts of this Readme were adopted from TI example i2c_controller_rw_multibyte_fifo_poll.