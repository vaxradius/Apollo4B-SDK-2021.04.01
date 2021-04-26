Name:
=====
 nemadc_qspi_test


Description:
============
 NemaDC example.


This example demonstrates how to drive display panel with Quad-SPI interface.

Quad-SPI interface includes 6 signals,
* Chip select (CSX)
* SPI clock (CLK)
* Data interface 0 (DATA0)
* Data interface 1 (DATA1).
* Data interface 2 (DATA2).
* Data interface 3 (DATA3).

Quad-SPI adds two more I/O lines (DATA2 and DATA3) and sends 4 data bits per
clock cycle. During the write sequence the display controller writes one or
more bytes of information to the display module via the interface. The write
sequence is initiated when CSX is driven from high to low and ends when CSX
is pulled high. In this example, when send commands, SPI interface works at
SPI4 mode. When send frame data, SPI interface works at Qual-SPI mode. Panel
must be set to Quad-SPI mode through configuring related pins to correct H/L
level.

When define TESTMODE_EN to 1 in nemadc_qspi_test.c, this example runs at test pattern mode.
When define TESTMODE_EN to 0, this example runs at image display mode.


******************************************************************************


