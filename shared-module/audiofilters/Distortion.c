// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Cooper Dalrymple
//
// SPDX-License-Identifier: MIT

#include <stdint.h>
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>
#include "shared-bindings/audiofilters/Distortion.h"
#include "shared-module/audiofilters/Distortion.h"

/**
 * Based on Godot's AudioEffectDistortion
 * - https://docs.godotengine.org/en/stable/classes/class_audioeffectdistortion.html
 * - https://github.com/godotengine/godot/blob/master/servers/audio/effects/audio_effect_distortion.cpp
 */

void common_hal_audiofilters_distortion_construct(audiofilters_distortion_obj_t *self,
    mp_obj_t drive, mp_obj_t pre_gain, mp_obj_t post_gain,
    audiofilters_distortion_mode mode, mp_obj_t mix,
    uint32_t buffer_size, uint8_t bits_per_sample, bool samples_signed,
    uint8_t channel_count, uint32_t sample_rate) {

    // Basic settings every effect and audio sample has
    // These are the effects values, not the source sample(s)
    self->bits_per_sample = bits_per_sample; // Most common is 16, but 8 is also supported in many places
    self->samples_signed = samples_signed; // Are the samples we provide signed (common is true)
    self->channel_count = channel_count; // Channels can be 1 for mono or 2 for stereo
    self->sample_rate = sample_rate; // Sample rate for the effect, this generally needs to match all audio objects

    // To smooth things out as CircuitPython is doing other tasks most audio objects have a buffer
    // A double buffer is set up here so the audio output can use DMA on buffer 1 while we
    // write to and create buffer 2.
    // This buffer is what is passed to the audio component that plays the effect.
    // Samples are set sequentially. For stereo audio they are passed L/R/L/R/...
    self->buffer_len = buffer_size; // in bytes

    self->buffer[0] = m_malloc(self->buffer_len);
    if (self->buffer[0] == NULL) {
        common_hal_audiofilters_distortion_deinit(self);
        m_malloc_fail(self->buffer_len);
    }
    memset(self->buffer[0], 0, self->buffer_len);

    self->buffer[1] = m_malloc(self->buffer_len);
    if (self->buffer[1] == NULL) {
        common_hal_audiofilters_distortion_deinit(self);
        m_malloc_fail(self->buffer_len);
    }
    memset(self->buffer[1], 0, self->buffer_len);

    self->last_buf_idx = 1; // Which buffer to use first, toggle between 0 and 1

    // Initialize other values most effects will need.
    self->sample = NULL; // The current playing sample
    self->sample_remaining_buffer = NULL; // Pointer to the start of the sample buffer we have not played
    self->sample_buffer_length = 0; // How many samples do we have left to play (these may be 16 bit!)
    self->loop = false; // When the sample is done do we loop to the start again or stop (e.g. in a wav file)
    self->more_data = false; // Is there still more data to read from the sample or did we finish

    // The below section sets up the effect's starting values.

    // If we did not receive a BlockInput we need to create a default float value
    if (drive == MP_OBJ_NULL) {
        drive = mp_obj_new_float(0.0);
    }
    synthio_block_assign_slot(drive, &self->drive, MP_QSTR_drive);

    // If we did not receive a BlockInput we need to create a default float value
    if (pre_gain == MP_OBJ_NULL) {
        pre_gain = mp_obj_new_float(0.0);
    }
    synthio_block_assign_slot(pre_gain, &self->pre_gain, MP_QSTR_pre_gain);

    // If we did not receive a BlockInput we need to create a default float value
    if (post_gain == MP_OBJ_NULL) {
        post_gain = mp_obj_new_float(0.0);
    }
    synthio_block_assign_slot(post_gain, &self->post_gain, MP_QSTR_post_gain);

    self->mode = mode;

    // If we did not receive a BlockInput we need to create a default float value
    if (mix == MP_OBJ_NULL) {
        mix = mp_obj_new_float(1.0);
    }
    synthio_block_assign_slot(mix, &self->mix, MP_QSTR_mix);
}

bool common_hal_audiofilters_distortion_deinited(audiofilters_distortion_obj_t *self) {
    if (self->buffer[0] == NULL) {
        return true;
    }
    return false;
}

void common_hal_audiofilters_distortion_deinit(audiofilters_distortion_obj_t *self) {
    if (common_hal_audiofilters_distortion_deinited(self)) {
        return;
    }
    self->buffer[0] = NULL;
    self->buffer[1] = NULL;
}

mp_obj_t common_hal_audiofilters_distortion_get_drive(audiofilters_distortion_obj_t *self) {
    return self->drive.obj;
}

void common_hal_audiofilters_distortion_set_drive(audiofilters_distortion_obj_t *self, mp_obj_t arg) {
    synthio_block_assign_slot(arg, &self->drive, MP_QSTR_drive);
}

mp_obj_t common_hal_audiofilters_distortion_get_pre_gain(audiofilters_distortion_obj_t *self) {
    return self->pre_gain.obj;
}

void common_hal_audiofilters_distortion_set_pre_gain(audiofilters_distortion_obj_t *self, mp_obj_t arg) {
    synthio_block_assign_slot(arg, &self->pre_gain, MP_QSTR_pre_gain);
}

mp_obj_t common_hal_audiofilters_distortion_get_post_gain(audiofilters_distortion_obj_t *self) {
    return self->post_gain.obj;
}

void common_hal_audiofilters_distortion_set_post_gain(audiofilters_distortion_obj_t *self, mp_obj_t arg) {
    synthio_block_assign_slot(arg, &self->post_gain, MP_QSTR_post_gain);
}

audiofilters_distortion_mode common_hal_audiofilters_distortion_get_mode(audiofilters_distortion_obj_t *self) {
    return self->mode;
}

void common_hal_audiofilters_distortion_set_mode(audiofilters_distortion_obj_t *self, audiofilters_distortion_mode arg) {
    self->mode = arg;
}

mp_obj_t common_hal_audiofilters_distortion_get_mix(audiofilters_distortion_obj_t *self) {
    return self->mix.obj;
}

void common_hal_audiofilters_distortion_set_mix(audiofilters_distortion_obj_t *self, mp_obj_t arg) {
    synthio_block_assign_slot(arg, &self->mix, MP_QSTR_mix);
}

uint32_t common_hal_audiofilters_distortion_get_sample_rate(audiofilters_distortion_obj_t *self) {
    return self->sample_rate;
}

uint8_t common_hal_audiofilters_distortion_get_channel_count(audiofilters_distortion_obj_t *self) {
    return self->channel_count;
}

uint8_t common_hal_audiofilters_distortion_get_bits_per_sample(audiofilters_distortion_obj_t *self) {
    return self->bits_per_sample;
}

void audiofilters_distortion_reset_buffer(audiofilters_distortion_obj_t *self,
    bool single_channel_output,
    uint8_t channel) {

    memset(self->buffer[0], 0, self->buffer_len);
    memset(self->buffer[1], 0, self->buffer_len);
}

bool common_hal_audiofilters_distortion_get_playing(audiofilters_distortion_obj_t *self) {
    return self->sample != NULL;
}

void common_hal_audiofilters_distortion_play(audiofilters_distortion_obj_t *self, mp_obj_t sample, bool loop) {
    // When a sample is to be played we must ensure the samples values matches what we expect
    // Then we reset the sample and get the first buffer to play
    // The get_buffer function will actually process that data

    if (audiosample_sample_rate(sample) != self->sample_rate) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("The sample's %q does not match"), MP_QSTR_sample_rate);
    }
    if (audiosample_channel_count(sample) != self->channel_count) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("The sample's %q does not match"), MP_QSTR_channel_count);
    }
    if (audiosample_bits_per_sample(sample) != self->bits_per_sample) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("The sample's %q does not match"), MP_QSTR_bits_per_sample);
    }
    bool single_buffer;
    bool samples_signed;
    uint32_t max_buffer_length;
    uint8_t spacing;
    audiosample_get_buffer_structure(sample, false, &single_buffer, &samples_signed, &max_buffer_length, &spacing);
    if (samples_signed != self->samples_signed) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("The sample's %q does not match"), MP_QSTR_signedness);
    }

    self->sample = sample;
    self->loop = loop;

    audiosample_reset_buffer(self->sample, false, 0);
    audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);

    // Track remaining sample length in terms of bytes per sample
    self->sample_buffer_length /= (self->bits_per_sample / 8);
    // Store if we have more data in the sample to retrieve
    self->more_data = result == GET_BUFFER_MORE_DATA;

    return;
}

void common_hal_audiofilters_distortion_stop(audiofilters_distortion_obj_t *self) {
    // When the sample is set to stop playing do any cleanup here
    self->sample = NULL;
    return;
}

static mp_float_t db_to_linear(mp_float_t value) {
	return expf(value * MICROPY_FLOAT_CONST(0.11512925464970228420089957273422));
}

audioio_get_buffer_result_t audiofilters_distortion_get_buffer(audiofilters_distortion_obj_t *self, bool single_channel_output, uint8_t channel,
    uint8_t **buffer, uint32_t *buffer_length) {

    if (!single_channel_output) {
        channel = 0;
    }

    // get the effect values we need from the BlockInput. These may change at run time so you need to do bounds checking if required
    mp_float_t drive = MIN(MAX(synthio_block_slot_get(&self->drive), 0.0), 1.0);
    mp_float_t pre_gain = db_to_linear(MIN(MAX(synthio_block_slot_get(&self->pre_gain), -60.0), 60.0));
    mp_float_t post_gain = db_to_linear(MIN(MAX(synthio_block_slot_get(&self->post_gain), -80.0), 24.0));
    mp_float_t mix = MIN(MAX(synthio_block_slot_get(&self->mix), 0.0), 1.0);

    // Switch our buffers to the other buffer
    self->last_buf_idx = !self->last_buf_idx;

    // If we are using 16 bit samples we need a 16 bit pointer, 8 bit needs an 8 bit pointer
    int16_t *word_buffer = (int16_t *)self->buffer[self->last_buf_idx];
    int8_t *hword_buffer = self->buffer[self->last_buf_idx];
    uint32_t length = self->buffer_len / (self->bits_per_sample / 8);

    // Loop over the entire length of our buffer to fill it, this may require several calls to get data from the sample
    while (length != 0) {
        // Check if there is no more sample to play, we will either load more data, reset the sample if loop is on or clear the sample
        if (self->sample_buffer_length == 0) {
            if (!self->more_data) { // The sample has indicated it has no more data to play
                if (self->loop && self->sample) { // If we are supposed to loop reset the sample to the start
                    audiosample_reset_buffer(self->sample, false, 0);
                } else { // If we were not supposed to loop the sample, stop playing it
                    self->sample = NULL;
                }
            }
            if (self->sample) {
                // Load another sample buffer to play
                audioio_get_buffer_result_t result = audiosample_get_buffer(self->sample, false, 0, (uint8_t **)&self->sample_remaining_buffer, &self->sample_buffer_length);
                // Track length in terms of words.
                self->sample_buffer_length /= (self->bits_per_sample / 8);
                self->more_data = result == GET_BUFFER_MORE_DATA;
            }
        }

        // If we have a sample, filter it
        if (self->sample != NULL) {
            // Determine how many bytes we can process to our buffer, the less of the sample we have left and our buffer remaining
            uint32_t n = MIN(self->sample_buffer_length, length);

            int16_t *sample_src = (int16_t *)self->sample_remaining_buffer; // for 16-bit samples
            int8_t *sample_hsrc = (int8_t *)self->sample_remaining_buffer; // for 8-bit samples

            if (mix <= 0.01) { // if mix is zero pure sample only
                for (uint32_t i = 0; i < n; i++) {
                    if (MP_LIKELY(self->bits_per_sample == 16)) {
                        word_buffer[i] = sample_src[i];
                    } else {
                        hword_buffer[i] = sample_hsrc[i];
                    }
                }
            } else {

                // Pre-calculate drive-based constants if needed by effect mode
                mp_float_t word_mult = 0;
                mp_float_t word_div = 0;
                switch (self->mode) {
                    case DISTORTION_MODE_ATAN:
                        word_mult = powf(10.0, drive * drive * 3.0) - 1.0 + 0.001;
                        word_div = 1.0 / (atanf(word_mult) * (1.0 + drive * 8));
                        break;
                    case DISTORTION_MODE_LOFI:
                        word_mult = powf(2.0, 2.0 + (1.0 - drive) * 14); // goes from 16 to 2 bits
                        break;
                    default:
                        break;
                }

                for (uint32_t i = 0; i < n; i++) {
                    int32_t sample_word = 0;
                    if (MP_LIKELY(self->bits_per_sample == 16)) {
                        sample_word = sample_src[i];
                    } else {
                        if (self->samples_signed) {
                            sample_word = sample_hsrc[i];
                        } else {
                            // Be careful here changing from an 8 bit unsigned to signed into a 32-bit signed
                            sample_word = (int8_t)(((uint8_t)sample_hsrc[i]) ^ 0x80);
                        }
                    }

                    int32_t word = sample_word * pre_gain;
                    switch (self->mode) {
                        case DISTORTION_MODE_CLIP: {
                            mp_float_t word_sign = word < 0 ? -1.0f : 1.0f;
                            word = powf(fabs(word / 32768.0), 1.0001 - drive) * word_sign * 32767.0;
                            word = MIN(MAX(word, -32767), 32768); // Hard clip
                        } break;
                        case DISTORTION_MODE_ATAN: {
                            word = atanf(word / 32768.0 * word_mult) * word_div * 32767.0;
                        } break;
                        case DISTORTION_MODE_LOFI: {
                            word = floorf(word / 32768.0 * word_mult + 0.5) / word_mult * 32767.0;
                        } break;
                        case DISTORTION_MODE_OVERDRIVE: {
                            mp_float_t x = word / 32768.0 * 0.686306;
                            mp_float_t z = 1 + expf(sqrtf(fabs(x)) * -0.75);
                            word = (expf(x) - expf(-x * z)) / (expf(x) + expf(-x)) * 32767.0;
                        } break;
                        case DISTORTION_MODE_WAVESHAPE: {
                            mp_float_t x = word / 32768.0;
                            mp_float_t k = 2 * drive / (1.00001 - drive);
                            word = (1.0 + k) * x / (1.0 + k * fabsf(x)) * 32767.0;
                        } break;
                    }
                    word = word * post_gain;
                    
                    if (MP_LIKELY(self->bits_per_sample == 16)) {
                        word_buffer[i] = (sample_word * (1.0 - mix)) + (word * mix);
                        if (!self->samples_signed) {
                            word_buffer[i] ^= 0x8000;
                        }
                    } else {
                        int8_t mixed = (sample_word * (1.0 - mix)) + (word * mix);
                        if (self->samples_signed) {
                            hword_buffer[i] = mixed;
                        } else {
                            hword_buffer[i] = (uint8_t)mixed ^ 0x80;
                        }
                    }
                }
            }

            // Update the remaining length and the buffer positions based on how much we wrote into our buffer
            length -= n;
            word_buffer += n;
            hword_buffer += n;
            self->sample_remaining_buffer += (n * (self->bits_per_sample / 8));
            self->sample_buffer_length -= n;
        }
    }

    // Finally pass our buffer and length to the calling audio function
    *buffer = (uint8_t *)self->buffer[self->last_buf_idx];
    *buffer_length = self->buffer_len;

    // Distortion always returns more data but some effects may return GET_BUFFER_DONE or GET_BUFFER_ERROR (see audiocore/__init__.h)
    return GET_BUFFER_MORE_DATA;
}

void audiofilters_distortion_get_buffer_structure(audiofilters_distortion_obj_t *self, bool single_channel_output,
    bool *single_buffer, bool *samples_signed, uint32_t *max_buffer_length, uint8_t *spacing) {

    // Return information about the effect's buffer (not the sample's)
    // These are used by calling audio objects to determine how to handle the effect's buffer
    *single_buffer = false;
    *samples_signed = self->samples_signed;
    *max_buffer_length = self->buffer_len;
    if (single_channel_output) {
        *spacing = self->channel_count;
    } else {
        *spacing = 1;
    }
}