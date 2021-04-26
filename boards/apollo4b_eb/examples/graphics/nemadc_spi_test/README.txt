Name:
=====
 nemadc_spi_test


Description:
============
 NemaDC example.


This example demonstrates how to drive a SPI4 panel.

4-wire SPI includes 4 signals,
* Chip select (CSX)
* SPI clock (CLK)
* SPI bidirectional data interface (DATA)
* Data and command switch (DCX).

During the write sequence the display controller writes one or more bytes of
information to the display module via the interface. The write sequence is
initiated when CSX is driven from high to low and ends when CSX is pulled high.
DCX is driven low while command information is on the interface and is pulled
high when data is present.

When define TESTMODE_EN to 1 in nemadc_spi_test.c, this example runs at test pattern mode.
When define TESTMODE_EN to 0, this example runs at image display mode.


******************************************************************************


