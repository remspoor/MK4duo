/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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
 * M100 Free Memory Watcher
 *
 * This code watches the free memory block between the bottom of the heap and the top of the stack.
 * This memory block is initialized and watched via the M100 command.
 *
 * M100 I   Initializes the free memory block and prints vitals statistics about the area
 *
 * M100 F   Identifies how much of the free memory block remains free and unused. It also
 *          detects and reports any corruption within the free memory block that may have
 *          happened due to errant firmware.
 *
 * M100 D   Does a hex display of the free memory block along with a flag for any errant
 *          data that does not match the expected value.
 *
 * M100 C x Corrupts x locations within the free memory block. This is useful to check the
 *          correctness of the M100 F and M100 D commands.
 *
 * Initial version by Roxy-3D
 */

#include "../../base.h"

#if ENABLED(M100_FREE_MEMORY_WATCHER)

  #define TEST_BYTE 0xE5

  extern char* __brkval;
  extern size_t  __heap_start, __heap_end, __flp;
  extern char __bss_end;

  #include "hex_print_routines.h"

  //
  // Utility functions
  //

  #define END_OF_HEAP() (__brkval ? __brkval : &__bss_end)

  // Location of a variable on its stack frame. Returns a value above
  // the stack (once the function returns to the caller).
  char* top_of_stack() {
    char x;
    return &x + 1; // x is pulled on return;
  }

  // Count the number of test bytes at the specified location.
  int16_t count_test_bytes(const char * const ptr) {
    for (uint16_t i = 0; i < 32000; i++)
      if (ptr[i] != TEST_BYTE)
        return i - 1;

    return -1;
  }

  // Return a count of free memory blocks.
  uint16_t free_memory_is_corrupted(char * const ptr, const uint16_t size) {
    // Find the longest block of test bytes in the given buffer
    uint16_t block_cnt = 0;
    for (uint16_t i = 0; i < size; i++) {
      if (ptr[i] == TEST_BYTE) {
        const uint16_t j = count_test_bytes(ptr + i);
        if (j > 8) {
          //SERIAL_MV("Found ", j);
          //SERIAL_EMV(" bytes free at 0x", hex_word((uint16_t)ptr + i));
          i += j;
          block_cnt++;
        }
      }
    }
    //if (block_cnt > 1) {
    //  SERIAL_EM("\nMemory Corruption detected in free memory area.");
    //  SERIAL_EMV("\nLargest free block is ", max_cnt);
    //}
    return block_cnt;
  }

  //
  // M100 sub-commands
  //
  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    /**
     * M100 D
     *  Dump the free memory block from __brkval to the stack pointer.
     *  malloc() eats memory from the start of the block and the stack grows
     *  up from the bottom of the block. Solid test bytes indicate nothing has
     *  used that memory yet. There should not be anything but test bytes within
     *  the block. If so, it may indicate memory corruption due to a bad pointer.
     *  Unexpected bytes are flagged in the right column.
     */
    void dump_free_memory(char *ptr, char *sp) {
      //
      // Start and end the dump on a nice 16 byte boundary
      // (even though the values are not 16-byte aligned).
      //
      ptr = (char*)((uint16_t)ptr & 0xFFF0); // Align to 16-byte boundary
      sp  = (char*)((uint16_t)sp  | 0x000F); // Align sp to the 15th byte (at or above sp)

      // Dump command main loop
      while (ptr < sp) {
        print_hex_word((uint16_t)ptr);      // Print the address
        SERIAL_C(':');
        for (uint8_t i = 0; i < 16; i++) {  // and 16 data bytes
          if (i == 8) SERIAL_C('-');
          print_hex_byte(ptr[i]);
          SERIAL_C(' ');
        }
        SERIAL_C('|');                   // Point out non test bytes
        for (uint8_t i = 0; i < 16; i++)
          SERIAL_C(ptr[i] == TEST_BYTE ? ' ' : '?');
        SERIAL_E;
        ptr += 16;
        idle();
      }
    }
  #endif // M100_FREE_MEMORY_DUMPER

  /**
   * M100 F
   *  Return the number of free bytes in the memory pool,
   *  with other vital statistics defining the pool.
   */
  void free_memory_pool_report(const char * const ptr, const uint16_t size) {
    int16_t max_cnt = -1;
    uint16_t block_cnt = 0;
    char *max_addr = NULL;
    // Find the longest block of test bytes in the buffer
    for (uint16_t i = 0; i < size; i++) {
      char * const addr = ptr + i;
      if (*addr == TEST_BYTE) {
        const uint16_t j = count_test_bytes(addr);
        if (j > 8) {
          SERIAL_MV("Found ", j);
          SERIAL_EMV(" bytes free at 0x", hex_word((uint16_t)addr));
          if (j > max_cnt) {
            max_cnt  = j;
            max_addr = addr;
          }
          i += j;
          block_cnt++;
        }
      }
    }
    if (block_cnt > 1) {
      SERIAL_M("\nMemory Corruption detected in free memory area.");
      SERIAL_MV("\nLargest free block is ", max_cnt);
      SERIAL_EMV(" bytes at 0x", hex_word((uint16_t)max_addr));
    }
    SERIAL_EMV("free_memory_is_corrupted() = ", free_memory_is_corrupted(ptr, size));
  }

  #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
    /**
     * M100 C<num>
     *  Corrupt <num> locations in the free memory pool and report the corrupt addresses.
     *  This is useful to check the correctness of the M100 D and the M100 F commands.
     */
    void corrupt_free_memory(char *ptr, const uint16_t size) {
      if (code_seen('C')) {
        ptr += 8;
        const uint16_t near_top = top_of_stack() - ptr - 250, // -250 to avoid interrupt activity that's altered the stack.
                       j = near_top / (size + 1);

        SERIAL_M("Corrupting free memory block.\n");
        for (uint16_t i = 1; i <= size; i++) {
          char * const addr = ptr + i * j;
          *addr = i;
          SERIAL_MV("\nCorrupting address: 0x", hex_word((uint16_t)addr));
        }
        SERIAL_E;
      }
    }
  #endif // M100_FREE_MEMORY_CORRUPTOR

  /**
   * M100 I
   *  Init memory for the M100 tests. (Automatically applied on the first M100.)
   */
  void init_free_memory(char *ptr, int16_t size) {
    SERIAL_M("Initializing free memory block.\n\n");

    size -= 250;    // -250 to avoid interrupt activity that's altered the stack.
    if (size < 0) return;

    ptr += 8;
    memset(ptr, TEST_BYTE, size);

    SERIAL_V(size);
    SERIAL_M(" bytes of memory initialized.\n");

    for (uint16_t i = 0; i < size; i++) {
      if (ptr[i] != TEST_BYTE) {
        SERIAL_MV("? address : 0x", hex_word((uint16_t)ptr + i));
        SERIAL_MV("=", hex_byte(ptr[i]));
        SERIAL_E; SERIAL_E;
      }
    }
  }

  /**
   * M100: Free Memory Check
   */
  void gcode_M100() {
    SERIAL_MV("\n__brkval : 0x", hex_word((uint16_t)__brkval));
    SERIAL_MV("\n__bss_end : 0x", hex_word((uint16_t)&__bss_end));

    char *ptr = END_OF_HEAP(), *sp = top_of_stack();

    SERIAL_MV("\nstart of free space : 0x", hex_word((uint16_t)ptr));
    SERIAL_EMV("\nStack Pointer : 0x", hex_word((uint16_t)sp));

    // Always init on the first invocation of M100
    static bool m100_not_initialized = true;
    if (m100_not_initialized || code_seen('I')) {
      m100_not_initialized = false;
      init_free_memory(ptr, sp - ptr);
    }

    #if ENABLED(M100_FREE_MEMORY_DUMPER)

      if (code_seen('D'))
        return dump_free_memory(ptr, sp);

    #endif

    if (code_seen('F'))
      return free_memory_pool_report(ptr, sp - ptr);

    #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)

      if (code_seen('C'))
        return corrupt_free_memory(ptr, code_value_int());

    #endif
  }

#endif // M100_FREE_MEMORY_WATCHER
