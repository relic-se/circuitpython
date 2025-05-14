// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Adafruit Industries LLC
//
// SPDX-License-Identifier: MIT

#include "shared-module/sdcardio/__init__.h"

#include "extmod/vfs_fat.h"

#include "shared-bindings/busio/SPI.h"
#include "shared-bindings/digitalio/DigitalInOut.h"
#include "shared-bindings/sdcardio/SDCard.h"

#include "supervisor/filesystem.h"

#if defined(DEFAULT_SD_CARD_DETECT) || CIRCUITPY_OS_GETENV
#ifndef DEFAULT_SD_CARD_DETECT
#include "shared-bindings/os/__init__.h"
#endif

static digitalio_digitalinout_obj_t sd_card_detect_pin;
static sdcardio_sdcard_obj_t sdcard;

static mp_vfs_mount_t _sdcard_vfs;
fs_user_mount_t _sdcard_usermount;

static bool _init_error = false;
static bool _mounted = false;

#if !defined(DEFAULT_SD_CARD_DETECT) || defined(DEFAULT_SD_MOSI)
static busio_spi_obj_t busio_spi_obj;
#else
#include "shared-bindings/board/__init__.h"
#endif
#endif

void sdcardio_init(void) {
    #if defined(DEFAULT_SD_CARD_DETECT) || CIRCUITPY_OS_GETENV
    #ifndef DEFAULT_SD_CARD_DETECT
    mp_obj_t arg = common_hal_os_getenv("CIRCUITPY_SD_CARD_DETECT", mp_const_none);
    if (arg != mp_const_none) {
        const mcu_pin_obj_t *pin = common_hal_digitalio_validate_pin(arg);
    #endif
    sd_card_detect_pin.base.type = &digitalio_digitalinout_type;
    #ifndef DEFAULT_SD_CARD_DETECT
    common_hal_digitalio_digitalinout_construct(&sd_card_detect_pin, pin);
    #else
    common_hal_digitalio_digitalinout_construct(&sd_card_detect_pin, DEFAULT_SD_CARD_DETECT);
    #endif
    common_hal_digitalio_digitalinout_switch_to_input(&sd_card_detect_pin, PULL_UP);
    common_hal_digitalio_digitalinout_never_reset(&sd_card_detect_pin);
    #ifndef DEFAULT_SD_CARD_DETECT
    }
    #endif
    #endif
}

void automount_sd_card(void) {
    #if defined(DEFAULT_SD_CARD_DETECT) || CIRCUITPY_OS_GETENV
    #ifndef DEFAULT_SD_CARD_DETECT
    mp_obj_t arg_detect = common_hal_os_getenv("CIRCUITPY_SD_CARD_DETECT", mp_const_none);
    mp_obj_t arg_mosi = common_hal_os_getenv("CIRCUITPY_SD_MOSI", mp_const_none);
    if (arg_detect == mp_const_none || arg_mosi == mp_const_none) {
        return;
    }

    mp_obj_t arg_sck = common_hal_os_getenv("CIRCUITPY_SD_SCK", mp_const_none);
    mp_obj_t arg_miso = common_hal_os_getenv("CIRCUITPY_SD_MISO", mp_const_none);
    mp_obj_t arg_cs = common_hal_os_getenv("CIRCUITPY_SD_CS", mp_const_none);
    
    const mcu_pin_obj_t *pin_sck = common_hal_digitalio_validate_pin(arg_sck);
    const mcu_pin_obj_t *pin_mosi = common_hal_digitalio_validate_pin(arg_mosi);
    const mcu_pin_obj_t *pin_miso = common_hal_digitalio_validate_pin(arg_miso);
    const mcu_pin_obj_t *pin_cs = common_hal_digitalio_validate_pin(arg_cs);
    
    bool inserted = false;
    mp_obj_t arg_inserted = common_hal_os_getenv("CIRCUITPY_SD_CARD_INSERTED", mp_const_none);
    if (arg_inserted != mp_const_none) {
        inserted = mp_obj_is_true(arg_inserted);
    }
    if (common_hal_digitalio_digitalinout_get_value(&sd_card_detect_pin) != inserted) {
    #else
    if (common_hal_digitalio_digitalinout_get_value(&sd_card_detect_pin) != DEFAULT_SD_CARD_INSERTED) {
    #endif
        // No card.
        _init_error = false;
        if (_mounted) {
            // Unmount the card.
            mp_vfs_mount_t *cur = MP_STATE_VM(vfs_mount_table);
            if (cur == &_sdcard_vfs) {
                MP_STATE_VM(vfs_mount_table) = cur->next;
            } else {
                while (cur->next != &_sdcard_vfs && cur != NULL) {
                    cur = cur->next;
                }
                if (cur != NULL) {
                    cur->next = _sdcard_vfs.next;
                }
            }
            _sdcard_vfs.next = NULL;

            #ifdef !defined(DEFAULT_SD_CARD_DETECT) || DEFAULT_SD_MOSI
            common_hal_busio_spi_deinit(&busio_spi_obj);
            #endif
            _mounted = false;
        }
        return;
    } else if (_init_error || _mounted) {
        // We've already tried and failed to init the card. Don't try again.
        return;
    }

    busio_spi_obj_t *spi_obj;
    #if defined(DEFAULT_SD_CARD_DETECT) && !defined(DEFAULT_SD_MOSI)
    spi_obj = MP_OBJ_TO_PTR(common_hal_board_create_spi(0));
    #elif !defined(DEFAULT_SD_CARD_DETECT) || defined(DEFAULT_SD_MOSI)
    spi_obj = &busio_spi_obj;
    spi_obj->base.type = &busio_spi_type;
    #ifdef DEFAULT_SD_MOSI
    common_hal_busio_spi_construct(spi_obj, DEFAULT_SD_SCK, DEFAULT_SD_MOSI, DEFAULT_SD_MISO, false);
    #else
    common_hal_busio_spi_construct(spi_obj, pin_sck, pin_mosi, pin_miso, false);
    #endif
    common_hal_busio_spi_never_reset(spi_obj);
    #endif
    sdcard.base.type = &sdcardio_SDCard_type;
    #ifndef DEFAULT_SD_CARD_DETECT
    mp_rom_error_text_t error = sdcardio_sdcard_construct(&sdcard, spi_obj, pin_cs, 25000000);
    #else
    mp_rom_error_text_t error = sdcardio_sdcard_construct(&sdcard, spi_obj, DEFAULT_SD_CS, 25000000);
    #endif
    if (error != NULL) {
        // Failed to communicate with the card.
        _mounted = false;
        _init_error = true;
        #if defined(DEFAULT_SD_MOSI) || !defined(DEFAULT_SD_CARD_DETECT)
        common_hal_busio_spi_deinit(spi_obj);
        #endif
        return;
    }
    common_hal_digitalio_digitalinout_never_reset(&sdcard.cs);

    fs_user_mount_t *vfs = &_sdcard_usermount;
    vfs->base.type = &mp_fat_vfs_type;
    vfs->fatfs.drv = vfs;

    // Initialise underlying block device
    vfs->blockdev.block_size = FF_MIN_SS; // default, will be populated by call to MP_BLOCKDEV_IOCTL_BLOCK_SIZE
    mp_vfs_blockdev_init(&vfs->blockdev, &sdcard);

    // mount the block device so the VFS methods can be used
    FRESULT res = f_mount(&vfs->fatfs);
    if (res != FR_OK) {
        _mounted = false;
        _init_error = true;
        common_hal_sdcardio_sdcard_deinit(&sdcard);
        #if defined(DEFAULT_SD_MOSI) || !defined(DEFAULT_SD_CARD_DETECT)
        common_hal_busio_spi_deinit(spi_obj);
        #endif
        return;
    }

    filesystem_set_concurrent_write_protection(vfs, true);
    filesystem_set_writable_by_usb(vfs, false);

    mp_vfs_mount_t *sdcard_vfs = &_sdcard_vfs;
    sdcard_vfs->str = "/sd";
    sdcard_vfs->len = 3;
    sdcard_vfs->obj = MP_OBJ_FROM_PTR(&_sdcard_usermount);
    sdcard_vfs->next = MP_STATE_VM(vfs_mount_table);
    MP_STATE_VM(vfs_mount_table) = sdcard_vfs;
    _mounted = true;
    #endif
}
