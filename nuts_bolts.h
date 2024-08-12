/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#ifndef nuts_bolts_h
#define nuts_bolts_h

#include "asf.h"
#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

#define SOME_LARGE_VALUE 1.0E+38

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 4 // Number of axes
#define X_AXIS 0 // Axis indexing value.
#define Y_AXIS 1
#define Z_AXIS 2
#define C_AXIS 3
// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#define TICKS_PER_MICROSECOND 150

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1


//void pio_enable_set_disable(uint32_t* port, uint32_t mask);


// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
//#define max(a,b) (((a) > (b)) ? (a) : (b))
//#define min(a,b) (((a) < (b)) ? (a) : (b))
#define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))

// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

uint8_t io_bit_is_true(Pio* p, uint32_t mask);
uint8_t io_bit_is_false(Pio* p, uint32_t mask);

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Non-blocking delay function used for general operation and suspend features.
void delay_sec(float seconds, uint8_t mode);


// util delay from arm_gcc develop function this names
//// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
//void delay_ms(uint16_t ms);
//
//// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
//void delay_us(uint32_t us);

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
float hypot_f(float x, float y);

float convert_delta_vector_to_unit_vector(float *vector);
float limit_value_by_axis_maximum(float *max_value, float *unit_vec);

typedef struct AVR_VAR{
	uint32_t STEP_DDR;
	uint32_t STEP_PORT;
	uint32_t DIRECTION_DDR;
	uint32_t DIRECTION_PORT;
	uint32_t STEPPERS_DISABLE_DDR;
	uint32_t STEPPERS_DISABLE_PORT;
	uint32_t LIMIT_DDR;
	uint32_t LIMIT_PIN;
	uint32_t LIMIT_PORT;

	uint32_t LIMIT_INT;
	uint32_t LIMIT_PCMSK;

	uint32_t CONTROL_DDR;
	uint32_t CONTROL_PIN;
	uint32_t CONTROL_PORT;

	uint32_t CONTROL_INT;
	uint32_t CONTROL_PCMSK;

	uint32_t SPINDLE_PWM_DDR;
	uint32_t SPINDLE_PWM_PORT;

	uint32_t SPINDLE_ENABLE_DDR;
	uint32_t SPINDLE_ENABLE_PORT;

	uint32_t SPINDLE_DIRECTION_DDR;
	uint32_t SPINDLE_DIRECTION_PORT;

	uint32_t COOLANT_FLOOD_DDR;
	uint32_t COOLANT_FLOOD_PORT;

	uint32_t COOLANT_MIST_DDR;
	uint32_t COOLANT_MIST_PORT;

	// Define probe switch input pin.
	uint32_t PROBE_DDR;
	uint32_t PROBE_PIN;
	uint32_t PROBE_PORT;

	uint32_t TCCR0A;
	uint32_t TCCR0B;

	uint32_t TCCR1A;
	uint32_t TCCR1B;
	uint32_t TIMSK1;
}AVR_VAR;

extern AVR_VAR avr_var;

#ifdef __cplusplus
}
#endif


#endif
