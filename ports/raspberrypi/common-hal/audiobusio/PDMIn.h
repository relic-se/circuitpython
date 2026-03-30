// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

#include "common-hal/microcontroller/Pin.h"
#include "bindings/rp2pio/StateMachine.h"
#include "shared-module/audiocore/__init__.h"

#include "extmod/vfs_fat.h"
#include "py/obj.h"

typedef struct {
    audiosample_base_t base;
    
    uint8_t *buffer[2];
    uint8_t last_buf_idx;
    uint32_t buffer_len; // max buffer in bytes

    uint8_t serializer;
    uint8_t clock_unit;
    uint8_t bytes_per_sample;
    rp2pio_statemachine_obj_t state_machine;
} audiobusio_pdmin_obj_t;

void pdmin_reset(void);

void pdmin_background(void);


// These are not available from Python because it may be called in an interrupt.
void audiobusio_pdmin_reset_buffer(audiobusio_pdmin_obj_t *self,
    bool single_channel_output,
    uint8_t channel);
audioio_get_buffer_result_t audiobusio_pdmin_get_buffer(audiobusio_pdmin_obj_t *self,
    bool single_channel_output,
    uint8_t channel,
    uint8_t **buffer,
    uint32_t *buffer_length);                                                      // length in bytes
