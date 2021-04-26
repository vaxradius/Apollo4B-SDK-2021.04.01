#!/usr/bin/env python3
# Utility to create image blobs for Corvette Secure Boot

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
def process(loadaddress, appFile, magicNum, output, chip):

    app_binarray = bytearray()
    # Open the file, and read it into an array of integers.
    with appFile as f_app:
        app_binarray.extend(f_app.read())
        f_app.close()

    encVal = 0
    if ((magicNum == AM_IMAGE_MAGIC_CUSTPATCH) or (magicNum == AM_IMAGE_MAGIC_NONSECURE) or (magicNum == AM_IMAGE_MAGIC_INFO0)):
        hdr_length  = AM_IMAGEHDR_SIZE_AUX;   #fixed header length
    else:
        am_print("magic number", hex(magicNum), " not supported", level=AM_PRINT_LEVEL_ERROR)
        return
    am_print("Header Size = ", hex(hdr_length))

    #generate mutable byte array for the header
    hdr_binarray = bytearray([0x00]*hdr_length);

    orig_app_length  = (len(app_binarray))
    am_print("original app_size ",hex(orig_app_length), "(",orig_app_length,")")

    am_print("load_address ",hex(loadaddress), "(",loadaddress,")")
    if (loadaddress & 0x3):
        am_print("load address needs to be word aligned", level=AM_PRINT_LEVEL_ERROR)
        return

    if (loadaddress & (FLASH_PAGE_SIZE - 1)):
        am_print("WARNING!!! - load address is not page aligned", level=AM_PRINT_LEVEL_ERROR)

    if (magicNum == AM_IMAGE_MAGIC_INFO0):
        if (orig_app_length & 0x3):
            am_print("INFO0 blob length needs to be multiple of 4", level=AM_PRINT_LEVEL_ERROR)
            return
        if ((loadaddress + orig_app_length) > INFO_SIZE_BYTES):
            am_print("INFO0 Offset and length exceed size", level=AM_PRINT_LEVEL_ERROR)
            return

    # Add Padding
    app_binarray = pad_to_block_size(app_binarray, 4, 0)
    
    app_length  = (len(app_binarray))
    am_print("app_size ",hex(app_length), "(",app_length,")")

    # Create Image blobs

    # w0
    blobLen = hdr_length + app_length
    w0 = (magicNum << 24) | ((encVal & 0x1) << 23) | blobLen

    am_print("w0 =", hex(w0))
    fill_word(hdr_binarray, 0, w0)
        
    # w2
    securityVal = 0
    am_print("Security Value ", hex(securityVal))
    w2 = ((securityVal << 24) & 0xff000000) 
    fill_word(hdr_binarray, 8, w2)
    am_print("w2 = ",hex(w2))
    

    if (magicNum == AM_IMAGE_MAGIC_INFO0):
        # Insert the INFO0 size and offset
        addrWord = ((orig_app_length>>2) << 16) | ((loadaddress>>2) & 0xFFFF)
        versionKeyWord = keys.INFO_KEY
    else:
        # Insert the application binary load address.
        addrWord = loadaddress
        # Initialize versionKeyWord
        versionKeyWord = 0

    am_print("addrWord = ",hex(addrWord))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_ADDR, addrWord)

    am_print("versionKeyWord = ",hex(versionKeyWord))
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_VERKEY, versionKeyWord)


    enc_binarray = hdr_binarray[AM_IMAGEHDR_START_ENCRYPT:hdr_length] + app_binarray


    # compute the CRC for the blob - this is done on a clear image
    crc = crc32(hdr_binarray[AM_IMAGEHDR_START_CRC:hdr_length] + app_binarray)
    am_print("crc =  ",hex(crc));
    w1 = crc
    fill_word(hdr_binarray, AM_IMAGEHDR_OFFSET_CRC, w1)

    # now output all three binary arrays in the proper order
    output = output + '.bin'
    am_print("Writing to file ", output)
    with open(output, mode = 'wb') as out:
        out.write(hdr_binarray[0:AM_IMAGEHDR_START_ENCRYPT])
        out.write(enc_binarray)


def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Generate Corvette Image Blob')

    parser.add_argument('--chipType', dest='chip', type=str, default='apollo4a',
                        choices = ['apollo4a'],
                        help='Chip Type: apollo4a (default = apollo4a)')
  
    parser.add_argument('--bin', dest='appFile', type=argparse.FileType('rb'),
                        help='binary file (blah.bin)')
                        
    parser.add_argument('--load-address', dest='loadaddress', type=auto_int, default=hex(AM_SECBOOT_DEFAULT_NONSECURE_MAIN),
                        help='Load address of the binary.')

    parser.add_argument('--magic-num', dest='magic_num', default=hex(AM_IMAGE_MAGIC_NONSECURE),
                        type=lambda x: x.lower(),
                        choices = [
                                hex(AM_IMAGE_MAGIC_CUSTPATCH),
                                hex(AM_IMAGE_MAGIC_NONSECURE),
                                hex(AM_IMAGE_MAGIC_INFO0)
                                ],
                        help = 'Magic Num ('
                                + str(hex(AM_IMAGE_MAGIC_CUSTPATCH)) + ': CustOTA, '
                                + str(hex(AM_IMAGE_MAGIC_NONSECURE)) + ': NonSecure, '
                                + str(hex(AM_IMAGE_MAGIC_INFO0)) + ': Info0) '
                                '- default[Main]'
                                )
                                                    
    parser.add_argument('-o', dest = 'output', default='outimage',
                        help = 'Output filename (without the extension)')

    parser.add_argument('--loglevel', dest='loglevel', type=auto_int, default=AM_PRINT_LEVEL_INFO,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)


    args = parser.parse_args()
    args.magic_num = int(args.magic_num, 16)


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


    process(args.loadaddress, args.appFile, args.magic_num, args.output, args.chip)

if __name__ == '__main__':
    main()
