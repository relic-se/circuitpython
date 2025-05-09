// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "Adafruit Feather RP2350 Adalogger"
#define MICROPY_HW_MCU_NAME "rp2350a"

#define MICROPY_HW_NEOPIXEL (&pin_GPIO21)

#define DEFAULT_I2C_BUS_SCL (&pin_GPIO3)
#define DEFAULT_I2C_BUS_SDA (&pin_GPIO2)

#define DEFAULT_SPI_BUS_SCK (&pin_GPIO22)
#define DEFAULT_SPI_BUS_MOSI (&pin_GPIO23)
#define DEFAULT_SPI_BUS_MISO (&pin_GPIO20)

#define DEFAULT_UART_BUS_RX (&pin_GPIO1)
#define DEFAULT_UART_BUS_TX (&pin_GPIO0)

#define CIRCUITPY_PSRAM_CHIP_SELECT (&pin_GPIO8)

#define DEFAULT_SD_SCK (&pin_GPIO14)
#define DEFAULT_SD_MOSI (&pin_GPIO15)
#define DEFAULT_SD_MISO (&pin_GPIO16)
#define DEFAULT_SD_CS (&pin_GPIO19)
#define DEFAULT_SD_CARD_DETECT (&pin_GPIO13)
#define DEFAULT_SD_CARD_INSERTED false
