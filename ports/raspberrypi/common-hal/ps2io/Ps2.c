// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Cooper Dalrymple
//
// SPDX-License-Identifier: MIT

#include <stdint.h>

#include "py/runtime.h"
#include "common-hal/ps2io/Ps2.h"
#include "shared-bindings/ps2io/Ps2.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "bindings/rp2pio/StateMachine.h"

const uint16_t ps2_program_instructions[] = {
            //     .wrap_target
    0x00ea, //  0: jmp    !osre, 10
    0x00c0, //  1: jmp    pin, 0
    0x4001, //  2: in     pins, 1
    0x20a1, //  3: wait   1 pin, 1
    0xe029, //  4: set    x, 9
    0x2121, //  5: wait   0 pin, 1               [1]
    0x4001, //  6: in     pins, 1
    0x20a1, //  7: wait   1 pin, 1
    0x0045, //  8: jmp    x--, 5
    0x0000, //  9: jmp    0
    0xe000, // 10: set    pins, 0
    0xee82, // 11: set    pindirs, 2             [14]
    0xe181, // 12: set    pindirs, 1             [1]
    0xe028, // 13: set    x, 8                       
    0x2121, // 14: wait   0 pin, 1               [1]
    0x6001, // 15: out    pins, 1
    0x20a1, // 16: wait   1 pin, 1
    0x004e, // 17: jmp    x--, 14
    0x2121, // 18: wait   0 pin, 1               [1]
    0xe001, // 19: set    pins, 1
    0x20a0, // 20: wait   1 pin, 0
    0x2020, // 21: wait   0 pin, 0
    0xe080, // 22: set    pindirs, 0
    0x25a0, // 23: wait   1 pin, 0               [5]
            //     .wrap
};

void common_hal_ps2io_ps2_construct(ps2io_ps2_obj_t *self,
    const mcu_pin_obj_t *data_pin, const mcu_pin_obj_t *clock_pin) {

    if (data_pin->number != clock_pin->number - 1) {
        mp_raise_ValueError(MP_ERROR_TEXT("Data pin must precede clock pin"));
    }

    // Use the state machine to manage pins.
    common_hal_rp2pio_statemachine_construct(
        &self->state_machine,
        ps2_program_instructions, MP_ARRAY_SIZE(ps2_program_instructions),
        16700 * 8, // 16.7 khz clock with 8 cycles per clock
        NULL, 0, // init
        NULL, 0, // may_exec
        data_pin, 1, PIO_PINMASK32_NONE, PIO_PINMASK32_ALL, // out pin
        data_pin, 2, // in pins
        PIO_PINMASK32_FROM_VALUE(3), PIO_PINMASK32_NONE, // in pulls
        data_pin, 2, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // set pins
        NULL, 0, false, PIO_PINMASK32_NONE, PIO_PINMASK32_NONE, // sideset pins
        false, // No sideset enable
        clock_pin, PULL_UP, // jump pin
        PIO_PINMASK_FROM_PIN(clock_pin->number), // wait gpio pins
        true, // exclusive pin use
        true, 8, true, // shift out right
        false, // Wait for txstall
        true, 10, true, // in settings
        false, // Not user-interruptible.
        0, -1, // wrap settings
        PIO_ANY_OFFSET,
        PIO_FIFO_TYPE_DEFAULT,
        PIO_MOV_STATUS_DEFAULT,
        PIO_MOV_N_DEFAULT
        );
}

void common_hal_ps2io_ps2_deinit(ps2io_ps2_obj_t *self) {
    if (common_hal_ps2io_ps2_deinited(self)) {
        return;
    }

    common_hal_rp2pio_statemachine_deinit(&self->state_machine);
}

bool common_hal_ps2io_ps2_deinited(ps2io_ps2_obj_t *self) {
    return common_hal_rp2pio_statemachine_deinited(&self->state_machine);
}

uint16_t common_hal_ps2io_ps2_get_len(ps2io_ps2_obj_t *self) {
    return (uint16_t) self->state_machine.pending_buffers_read;
}

int16_t common_hal_ps2io_ps2_popleft(ps2io_ps2_obj_t *self) {
    if (common_hal_rp2pio_statemachine_get_rxstall(&self->state_machine)) {
        // buffer overflow, newest data discarded
        self->errors |= 0x10;
    }

    int16_t value;
    if (!common_hal_rp2pio_statemachine_readinto(&self->state_machine, (uint8_t *)&value, sizeof(int16_t), sizeof(int16_t), false)) {
        return -1;
    }

    if (value & (1 << 10)) {
        // start bit not 0
        self->errors |= 0x01;
        return -1;
    }

    if (!(value & 1)) {
        // stop bit not 1
        self->errors |= 0x08;
        return -1;
    }

    // verify odd parity
    bool parity = (bool)(value & 2);
    value = (value >> 2) & 0xff;
    uint8_t ones = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (value & (1 << i)) {
            ones++;
        }
    }
    if ((ones + parity) % 2 == 0) {
        // parity bit error
        self->errors |= 0x04;
        return -1;
    }

    return value;
}

int16_t common_hal_ps2io_ps2_sendcmd(ps2io_ps2_obj_t *self, uint8_t b) {
    common_hal_rp2pio_statemachine_clear_rxfifo(&self->state_machine); // flush pending read data
    common_hal_rp2pio_statemachine_write(&self->state_machine, &b, 1, 1, false);
    return common_hal_ps2io_ps2_popleft(self); // read response byte, typically ack (0xfa)
}

uint16_t common_hal_ps2io_ps2_clear_errors(ps2io_ps2_obj_t *self) {
    uint16_t errors = self->errors;
    self->errors = 0;
    return errors;
}
