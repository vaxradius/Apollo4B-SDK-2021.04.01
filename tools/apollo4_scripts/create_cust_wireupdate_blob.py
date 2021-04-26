#!/usr/bin/env python3
# Utility to generate image blobs for Corvette Bootloader assisted Wired updates

import argparse
import sys
import array
import os
import binascii
import importlib

from am_defines import *

#******************************************************************************
#
# Generate the image blob as per command line parameters
#
#******************************************************************************
def process(appFile, imagetype, loadaddress, optionsVal, maxSize, output, chip):

    app_binarray = bytearray()
    # Open the file, and read it into an array of integers.
    with appFile as f_app:
        app_binarray.extend(f_app.read())
        f_app.close()

    # Make sure it is page multiple
    if ((maxSize & (FLASH_PAGE_SIZE - 1)) != 0):
        am_print ("split needs to be multiple of flash page size", level=AM_PRINT_LEVEL_ERROR)
        return

    import keys as keys
    hdr_length  = AM_WU_IMAGEHDR_SIZE;   #fixed header length
    am_print("Header Size = ", hex(hdr_length))

    orig_app_length  = (len(app_binarray))

    # Add Padding
    app_binarray = pad_to_block_size(app_binarray, 4, 0)
    
    app_length  = (len(app_binarray))
    am_print("app_size ",hex(app_length), "(",app_length,")")

    if (app_length + hdr_length > maxSize):
        am_print("Image size bigger than max - Creating Split image")

    start = 0
    # now output all three binary arrays in the proper order
    output = output + '.bin'
    out = open(output, mode = 'wb')

    while (start < app_length):
        #generate mutable byte array for the header
        hdr_binarray = bytearray([0x00]*hdr_length);

        if (app_length - start > maxSize):
            end = start + maxSize
        else:
            end = app_length

        if (imagetype == AM_SECBOOT_WIRED_IMAGETYPE_INFO0_NOOTA):
            key = keys.INFO_KEY
            # word offset
            fill_word(hdr_binarray, AM_WU_IMAGEHDR_OFFSET_ADDR, loadaddress>>2)
        else:
            key = keys.FLASH_KEY
            # load address
            fill_word(hdr_binarray, AM_WU_IMAGEHDR_OFFSET_ADDR, loadaddress)
            if (loadaddress & (FLASH_PAGE_SIZE - 1)):
                am_print("WARNING!!! - load address is not page aligned", level=AM_PRINT_LEVEL_ERROR)

        # Create imageType & options
        hdr_binarray[AM_WU_IMAGEHDR_OFFSET_IMAGETYPE] = imagetype
        # Set the options only for the first block
        if (start == 0):
            hdr_binarray[AM_WU_IMAGEHDR_OFFSET_OPTIONS] = optionsVal
        else:
            hdr_binarray[AM_WU_IMAGEHDR_OFFSET_OPTIONS] = 0

        # Create Info0 Update Blob for wired update
        fill_word(hdr_binarray, AM_WU_IMAGEHDR_OFFSET_KEY, key)
        # update size
        fill_word(hdr_binarray, AM_WU_IMAGEHDR_OFFSET_SIZE, end-start)

        w0 = 0

        fill_word(hdr_binarray, 0, w0)

        enc_binarray = hdr_binarray[AM_WU_IMAGEHDR_START_ENCRYPT:hdr_length] + app_binarray[start:end]


        am_print("Writing to file ", output)
        am_print("Image from ", str(hex(start)), " to ", str(hex(end)), " will be loaded at", str(hex(loadaddress))) 
        out.write(hdr_binarray[0:AM_WU_IMAGEHDR_START_ENCRYPT])
        out.write(enc_binarray)

        # Reset start for next chunk
        start = end
        loadaddress = loadaddress + maxSize

def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Generate Corvette Wired Update Blob')
 
    parser.add_argument('--chipType', dest='chip', type=str, default='apollo4a',
                        choices = ['apollo4a'],
                        help='Chip Type: apollo4a (default = apollo4a)')

    parser.add_argument('--load-address', dest='loadaddress', type=auto_int, default=hex(0x60000),
                        help='Load address of the binary - Where in flash the blob will be stored (could be different than install address of binary within).')

    parser.add_argument('--bin', dest='appFile', type=argparse.FileType('rb'),
                        help='binary file (blah.bin)')
                        
    parser.add_argument('-i', dest = 'imagetype', default=AM_SECBOOT_WIRED_IMAGETYPE_MAIN, type=int,
                        choices = [
                                (AM_SECBOOT_WIRED_IMAGETYPE_SBL),
                                (AM_SECBOOT_WIRED_IMAGETYPE_AM3P),
                                (AM_SECBOOT_WIRED_IMAGETYPE_PATCH),
                                (AM_SECBOOT_WIRED_IMAGETYPE_CUSTPATCH),
                                (AM_SECBOOT_WIRED_IMAGETYPE_NONSECURE),
                                (AM_SECBOOT_WIRED_IMAGETYPE_INFO0),
                                (AM_SECBOOT_WIRED_IMAGETYPE_INFO0_NOOTA)
                                ],
                        help = 'ImageType ('
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_SBL) + ': SBL, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_AM3P) + ': AM3P, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_PATCH) + ': Patch, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_CUSTPATCH) + ': CustOTA, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_NONSECURE) + ': NonSecure, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_INFO0) + ': Info0, '
                                + str(AM_SECBOOT_WIRED_IMAGETYPE_INFO0_NOOTA) + ': Info0-NoOTA) '
                                '- default[Main]')

    parser.add_argument('--options', dest = 'options', type=auto_int, default=0x1,
                        help = 'Options (16b hex value) - bit0 instructs to perform OTA of the image after wired download (set to 0 if only downloading & skipping OTA flow)')

    parser.add_argument('-o', dest = 'output', default='wuimage',
                        help = 'Output filename (without the extension)')

    parser.add_argument('--split', dest='split', type=auto_int, default=hex(MAX_DOWNLOAD_SIZE),
                        help='Specify the max block size if the image will be downloaded in pieces')

    parser.add_argument('--loglevel', dest='loglevel', type=auto_int, default=AM_PRINT_LEVEL_INFO,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)


    args = parser.parse_args()

    return args

#******************************************************************************
#
# Main function.
#
#******************************************************************************
def main():
    # Read the arguments.
    args = parse_arguments()

    am_set_print_level(args.loglevel)
    process(args.appFile, args.imagetype, args.loadaddress, args.options, args.split, args.output, args.chip)

if __name__ == '__main__':
    main()
