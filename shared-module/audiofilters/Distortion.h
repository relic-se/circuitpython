// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
//
// SPDX-License-Identifier: MIT
#pragma once

#include "py/obj.h"

#include "shared-bindings/audiofilters/Distortion.h"
#include "shared-module/audiocore/__init__.h"
#include "shared-module/synthio/block.h"

typedef enum {
    DISTORTION_MODE_CLIP,
    DISTORTION_MODE_ATAN,
    DISTORTION_MODE_LOFI,
    DISTORTION_MODE_OVERDRIVE,
    DISTORTION_MODE_WAVESHAPE,
} audiofilters_distortion_mode;

extern const mp_obj_type_t audiofilters_distortion_type;

typedef struct {
    mp_obj_base_t base;
    synthio_block_slot_t drive;
    synthio_block_slot_t pre_gain;
    synthio_block_slot_t post_gain;
    audiofilters_distortion_mode mode;
    synthio_block_slot_t mix;

    uint8_t bits_per_sample;
    bool samples_signed;
    uint8_t channel_count;
    uint32_t sample_rate;

    int8_t *buffer[2];
    uint8_t last_buf_idx;
    uint32_t buffer_len; // max buffer in bytes

    uint8_t *sample_remaining_buffer;
    uint32_t sample_buffer_length;

    bool loop;
    bool more_data;

    mp_obj_t sample;
} audiofilters_distortion_obj_t;

void audiofilters_distortion_reset_buffer(audiofilters_distortion_obj_t *self,
    bool single_channel_output,
    uint8_t channel);

audioio_get_buffer_result_t audiofilters_distortion_get_buffer(audiofilters_distortion_obj_t *self,
    bool single_channel_output, uint8_t channel,
    uint8_t **buffer, uint32_t *buffer_length);

void audiofilters_distortion_get_buffer_structure(audiofilters_distortion_obj_t *self,
    bool single_channel_output, bool *single_buffer, bool *samples_signed,
    uint32_t *max_buffer_length, uint8_t *spacing);