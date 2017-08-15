/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../base.h"

#if ENABLED(M100_FREE_MEMORY_WATCHER) || ENABLED(DEBUG_GCODE_PARSER)

static char _hex[7] = "0x0000";

char* hex_byte(const uint8_t b) {
  _hex[4] = hex_nybble(b >> 4);
  _hex[5] = hex_nybble(b);
  return &_hex[4];
}

char* hex_word(const uint16_t w) {
  _hex[2] = hex_nybble(w >> 12);
  _hex[3] = hex_nybble(w >> 8);
  _hex[4] = hex_nybble(w >> 4);
  _hex[5] = hex_nybble(w);
  return &_hex[2];
}

char* hex_address(const void * const w) {
  (void)hex_word((int)w);
  return _hex;
}

void print_hex_nybble(const uint8_t n)        { SERIAL_CHR(hex_nybble(n));  }
void print_hex_byte(const uint8_t b)          { SERIAL_VAL(hex_byte(b));    }
void print_hex_word(const uint16_t w)         { SERIAL_VAL(hex_word(w));    }
void print_hex_address(const void * const w)  { SERIAL_VAL(hex_address(w)); }

#endif // M100_FREE_MEMORY_WATCHER
