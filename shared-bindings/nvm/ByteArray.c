// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include "py/binary.h"
#include "py/runtime.h"
#include "py/runtime0.h"
#include "shared-bindings/nvm/ByteArray.h"

//| class ByteArray:
//|     r"""Presents a stretch of non-volatile memory as a bytearray.
//|
//|     Non-volatile memory is available as a byte array that persists over reloads
//|     and power cycles. Each assignment causes an erase and write cycle so its recommended to assign
//|     all values to change at once.
//|
//|     Usage::
//|
//|        import microcontroller
//|        microcontroller.nvm[0:3] = b"\xcc\x10\x00"
//|     """
//|

//|     def __init__(self) -> None:
//|         """Not currently dynamically supported. Access the sole instance through `microcontroller.nvm`."""
//|         ...
//|

//|     def __bool__(self) -> bool: ...
//|     def __len__(self) -> int:
//|         """Return the length. This is used by (`len`)"""
//|         ...
//|
static mp_obj_t nvm_bytearray_unary_op(mp_unary_op_t op, mp_obj_t self_in) {
    nvm_bytearray_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint16_t len = common_hal_nvm_bytearray_get_length(self);
    switch (op) {
        case MP_UNARY_OP_BOOL:
            return mp_obj_new_bool(len != 0);
        case MP_UNARY_OP_LEN:
            return MP_OBJ_NEW_SMALL_INT(len);
        default:
            return MP_OBJ_NULL;      // op not supported
    }
}

static const mp_rom_map_elem_t nvm_bytearray_locals_dict_table[] = {
};

static MP_DEFINE_CONST_DICT(nvm_bytearray_locals_dict, nvm_bytearray_locals_dict_table);

//|     @overload
//|     def __getitem__(self, index: slice) -> bytearray: ...
//|     @overload
//|     def __getitem__(self, index: int) -> int:
//|         """Returns the value at the given index."""
//|         ...
//|
//|     @overload
//|     def __setitem__(self, index: slice, value: ReadableBuffer) -> None: ...
//|     @overload
//|     def __setitem__(self, index: int, value: int) -> None:
//|         """Set the value at the given index."""
//|         ...
//|
//|
static mp_obj_t nvm_bytearray_subscr(mp_obj_t self_in, mp_obj_t index_in, mp_obj_t value) {
    if (value == MP_OBJ_NULL) {
        // delete item
        // slice deletion
        return MP_OBJ_NULL; // op not supported
    } else {
        nvm_bytearray_obj_t *self = MP_OBJ_TO_PTR(self_in);
        if (0) {
        #if MICROPY_PY_BUILTINS_SLICE
        } else if (mp_obj_is_type(index_in, &mp_type_slice)) {
            mp_bound_slice_t slice;
            if (!mp_seq_get_fast_slice_indexes(common_hal_nvm_bytearray_get_length(self), index_in, &slice)) {
                mp_raise_NotImplementedError(MP_ERROR_TEXT("only slices with step=1 (aka None) are supported"));
            }
            if (value != MP_OBJ_SENTINEL) {
                #if MICROPY_PY_ARRAY_SLICE_ASSIGN
                // Assign
                size_t src_len = slice.stop - slice.start;
                uint8_t *src_items;
                if (mp_obj_is_type(value, &mp_type_array) ||
                    mp_obj_is_type(value, &mp_type_bytearray) ||
                    mp_obj_is_type(value, &mp_type_memoryview) ||
                    mp_obj_is_type(value, &mp_type_bytes)) {
                    mp_buffer_info_t bufinfo;
                    mp_get_buffer_raise(value, &bufinfo, MP_BUFFER_READ);
                    if (bufinfo.len != src_len) {
                        mp_raise_ValueError(MP_ERROR_TEXT("Slice and value different lengths."));
                    }
                    src_len = bufinfo.len;
                    src_items = bufinfo.buf;
                    if (1 != mp_binary_get_size('@', bufinfo.typecode, NULL)) {
                        mp_raise_ValueError(MP_ERROR_TEXT("Array values should be single bytes."));
                    }
                } else {
                    mp_raise_NotImplementedError(MP_ERROR_TEXT("array/bytes required on right side"));
                }

                if (!common_hal_nvm_bytearray_set_bytes(self, slice.start, src_items, src_len)) {
                    mp_raise_RuntimeError(MP_ERROR_TEXT("Unable to write to nvm."));
                }
                return mp_const_none;
                #else
                return MP_OBJ_NULL; // op not supported
                #endif
            } else {
                // Read slice.
                size_t len = slice.stop - slice.start;
                uint8_t *items = m_new(uint8_t, len);
                common_hal_nvm_bytearray_get_bytes(self, slice.start, len, items);
                return mp_obj_new_bytearray_by_ref(len, items);
            }
        #endif
        } else {
            // Single index rather than slice.
            size_t index = mp_get_index(self->base.type, common_hal_nvm_bytearray_get_length(self),
                index_in, false);
            if (value == MP_OBJ_SENTINEL) {
                // load
                uint8_t value_out;
                common_hal_nvm_bytearray_get_bytes(self, index, 1, &value_out);
                return MP_OBJ_NEW_SMALL_INT(value_out);
            } else {
                // store
                mp_int_t byte_value = mp_obj_get_int(value);
                mp_arg_validate_int_range(byte_value, 0, 255, MP_QSTR_bytes);

                uint8_t short_value = byte_value;
                if (!common_hal_nvm_bytearray_set_bytes(self, index, &short_value, 1)) {
                    mp_raise_RuntimeError(MP_ERROR_TEXT("Unable to write to nvm."));
                }
                return mp_const_none;
            }
        }
    }
}

MP_DEFINE_CONST_OBJ_TYPE(
    nvm_bytearray_type,
    MP_QSTR_ByteArray,
    MP_TYPE_FLAG_NONE,
    locals_dict, &nvm_bytearray_locals_dict,
    subscr, nvm_bytearray_subscr,
    unary_op, nvm_bytearray_unary_op
    );
