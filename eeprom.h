/*
  eeprom.h - EEPROM methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef eeprom_h
#define eeprom_h

#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t here_is_your_eeprom_dude[]; // 255 bytes

unsigned char eeprom_get_char(uint8_t addr);
void eeprom_put_char(uint8_t addr, uint8_t new_value);
void memcpy_to_eeprom_with_checksum(uint8_t destination, char *source, uint8_t size);
uint8_t memcpy_from_eeprom_with_checksum(char *destination, uint8_t source, uint8_t size);


#ifdef __cplusplus
}
#endif


#endif
