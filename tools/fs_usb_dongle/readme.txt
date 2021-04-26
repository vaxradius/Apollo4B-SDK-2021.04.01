Getting Started:
1. Firmware update:
    Run firmware_updater.exe in the fs_usb_dongle\firmware_updater folder.
    Follow the guidance and load the latest dongle firmware to th dongle.
    (firmware used for updater is named dfu_release_binary_vxx.bin in the same folder)
    
2. Firmware update over SWD:
    Update the dongle MCUs' firmwares using SWD interface.
    Dongle MCU firmware provided in \fs_usb_dongle\dongle_firmware folder.
    F405RGT_CDC_DEVICE_Dongle_Download_vx.bin is the firmware for MCU1.
    F405RGT_CDC_HOST_Dongle_Download_vx.bin is the firmware for MCU2.
    (Both firmware shall be downloaded to MCU starting from address 0x08000000)

3. Test with dongle + Apollo4 SIP EB (Loopback as example)
    - Apollo4 Rev.A1 sample needs to be used for the test.
    - Connect EB's JLINK USB to PC to download firmware.
    - Connect EB's FALCON USB to the FS USB Dongle for the test.
    - Download the test binary to Apollo4 Rev.A1 on the EB.
       For loopback test: fs_usb_dongle\test_scripts\loopback\ap4_tinyusb_cdc_loopback_example.bin, starting from 0x0000000 address.
    - Reset the EB.
    - User can keep the FS USB dongle unplugged at this step.
    - Run ap4_cdc_loopback_test.exe under the fs_usb_dongle\test_scripts\loopback\ folder. (or the *.py script)
        When the test script prints "Looking for Ambiq FS Dongle", plug the dongle to the PC.
    - Test starts and user can hit Ctrl+C to stop the test.
