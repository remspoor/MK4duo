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

/**
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_COLOR_LEDS

  #define CODE_M150

  /**
   * M150: Set Status LED Color - Use R-U-B-W for R-G-B-W
   *
   * Always sets all 3 or 4 components. If a component is left out, set to 0.
   *
   * Examples:
   *
   *   M150 R255       ; Turn LED red
   *   M150 R255 U127  ; Turn LED orange (PWM only)
   *   M150            ; Turn LED off
   *   M150 R U B      ; Turn LED white
   *   M150 W          ; Turn LED white using a white LED
   *
   */
  inline void gcode_M150(void) {
    printer.set_led_color(
      parser.seen('R') ? (parser.has_value() ? parser.value_byte() : 255) : 0,
      parser.seen('U') ? (parser.has_value() ? parser.value_byte() : 255) : 0,
      parser.seen('B') ? (parser.has_value() ? parser.value_byte() : 255) : 0
      #if ENABLED(RGBW_LED) || ENABLED(NEOPIXEL_RGBW_LED)
        , parser.seen('W') ? (parser.has_value() ? parser.value_byte() : 255) : 0
      #endif
    );
  }

#endif // HAS_COLOR_LEDS
