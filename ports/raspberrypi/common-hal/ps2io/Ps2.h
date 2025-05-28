// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Cooper Dalrymple
//
// SPDX-License-Identifier: MIT

#pragma once

#include "common-hal/microcontroller/Pin.h"
#include "bindings/rp2pio/StateMachine.h"

#include "py/obj.h"

typedef struct {
    mp_obj_base_t base;
    uint8_t clock_pin;
    uint8_t data_pin;
    uint16_t errors;
    rp2pio_statemachine_obj_t state_machine;
} ps2io_ps2_obj_t;

void ps2_reset(void);

void ps2_background(void);
