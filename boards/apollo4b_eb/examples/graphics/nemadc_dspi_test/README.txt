Name:
=====
 nemadc_dspi_test


Description:
============
 NemaDC example.


This example demonstrates how to drive display panel with 1P1T 2-wire
Dual-SPI interface.

1P1T 2-wire Dual-SPI interface includes 4 signals,
* Chip select (CSX)
* SPI clock (CLK)
* Data interface 0 (DATA0)
* Data interface 1 (DATA1).

During the write sequence the display controller writes one or more bytes of
information to the display module via the interface. The write sequence is
initiated when CSX is driven from high to low and ends when CSX is pulled high.
Dual-SPI reuses SPI4 DCX as the second DATA signal (DATA1), and sends 4 data
bits per clock cycle. In this example, when send commands, SPI interface works
at SPI4 mode. When send frame data, SPI interface works at Dual-SPI mode. Panel
must be set to Dual-SPI mode through writing register in panel driver IC before
sending frame data.

When define TESTMODE_EN to 1 in nemadc_dspi_test.c, this example runs at test pattern mode.
When define TESTMODE_EN to 0, this example runs at image display mode.


******************************************************************************


