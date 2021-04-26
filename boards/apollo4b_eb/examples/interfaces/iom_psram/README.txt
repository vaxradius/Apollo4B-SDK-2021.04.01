Name:
=====
 iom_psram


Description:
============
 Example that demostrates IOM, connecting to a SPI PSRAM

PSRAM is initialized with a known pattern data using Blocking IOM Write.
This example starts a 1 second timer. At each 1 second period, it initiates
reading a fixed size block from the PSRAM device using Non-Blocking IOM
Read, and comparing againts the predefined pattern

Define one of PSRAM_DEVICE_ macros to select the PSRAM device

SWO is configured in 1M baud, 8-n-1 mode.


******************************************************************************


