/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h

#include "asf.h"
#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif




/* wolne io													// main connector
	pd26	pa3		pa2		pd24	pb1(tx)		pa18		pa12	pa14
	pc31	pa4		pc19	pa10	pb0(rx)		pd17		pa13	pa21
	pd30	pd22	pd11	pc31	pd16		pd18		pb13	pb4
	pa19	pd20	pd27	pb13	pd15		pd19		pb2		pa30
	pc13	pd21	pa6		pd0		pd19		pa5			pb3		pa28
	pc30	pd25	pa5		pb3		pd18		pa29		pa31	pa18
	pa17	pc9		pd30	pb2		pd27		pa1			pa23	pc12
	pc12	pa17	pd28	pa22	pd28		pa26		pa25	pc14

	pa1 pa2 pa3 pa4 pa5 pa6 pa10 pa12 pa13 pa14 pa17 pa18 pa19 pa21 pa22 pa23 pa25 pa26 pa28 pa29 pa30 pa31
	pb0 pb1 pb2 pb3 pb4 pb13
	pc9 pc12 pc13 pc14 pc19 pc30 pc31
	pd0 pd11 pd15 pd16 pd17 pd18 pd19 pd20 pd21 pd22 pd24 pd25 pd26 pd28 pd27 pd28 pd30
*/
#define X_STEP_BIT				PIO_PA1
#define Y_STEP_BIT				PIO_PA2
#define Z_STEP_BIT				PIO_PA3
#define C_STEP_BIT				PIO_PA4
#define STEP_MASK				( (X_STEP_BIT) | (Y_STEP_BIT) | (DUAL_STEP_BIT) | (Z_STEP_BIT) | (C_STEP_BIT) ) // All step bits

#define DUAL_STEP_BIT			PIO_PA5
#define X_DIRECTION_BIT			PIO_PA6
#define Y_DIRECTION_BIT			PIO_PA10
#define Z_DIRECTION_BIT			PIO_PA12
#define C_DIR_BIT				PIO_PA13
#define DIRECTION_MASK			( (X_DIRECTION_BIT) | (Y_DIRECTION_BIT) | (Z_DIRECTION_BIT) | (C_DIR_BIT) )	// All direction bits

#define STEPPERS_DISABLE_BIT	PIO_PA14
#define X_LIMIT_BIT				PIO_PA17
#define Y_LIMIT_BIT				PIO_PA18
#define Z_LIMIT_BIT				PIO_PA19
#define LIMIT_MASK				( (X_LIMIT_BIT) | (Y_LIMIT_BIT) | (Z_LIMIT_BIT) )

// J505
#define PIO_TXD0				PIO_PB1
#define PIO_TXR0				PIO_PB0

#define CONTROL_RESET_BIT       PIO_PA21
#define CONTROL_FEED_HOLD_BIT	PIO_PA22
#define CONTROL_CYCLE_START_BIT PIO_PA23
#define CONTROL_SAFETY_DOOR_BIT PIO_PA25
#define CONTROL_MASK			( (CONTROL_RESET_BIT) | (CONTROL_FEED_HOLD_BIT) | ( CONTROL_CYCLE_START_BIT) | (CONTROL_SAFETY_DOOR_BIT) )

#define SPINDLE_DIRECTION_BIT   PIO_PA26
#define COOLANT_MIST_BIT		PIO_PA28
#define PROBE_BIT				PIO_PA29
#define SPINDLE_PWM_BIT			PIO_PA30
#define SPINDLE_ENABLE_BIT		PIO_PA30
#define COOLANT_FLOOD_BIT		PIO_PA31

#define UNDEFINED	(0)



#define STEPPERS_DISABLE_MASK		STEPPERS_DISABLE_BIT
#define CONTROL_INVERT_MASK			CONTROL_MASK // May be re-defined to only invert certain control pins.
#define SPINDLE_PWM_MAX_VALUE		255 // Don't change. 328p fast PWM mode fixes top value as 255.
#define SPINDLE_PWM_OFF_VALUE		0
#define SPINDLE_PWM_RANGE			(SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
#define PROBE_MASK					PROBE_BIT

#ifndef SPINDLE_PWM_MIN_VALUE
#define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
#endif







	// Define serial port pins and interrupt vectors.
	// rozwi¹zaæ jednym przerwaniem


	// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
	//#define STEP_DDR        UNDEFINED
	//#define STEP_PORT       UNDEFINED

	//#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<DUAL_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits


	// Define step direction output pins. NOTE: All direction pins must be on the same port.
	//#define DIRECTION_DDR     UNDEFINED
	//#define DIRECTION_PORT    UNDEFINED
	//#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<C_DIR_BIT))	// All direction bits


	// Define stepper driver enable/disable output pin.
	//#define STEPPERS_DISABLE_DDR    UNDEFINED
	//#define STEPPERS_DISABLE_PORT   UNDEFINED
	//#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)


	// Define homing/hard limit switch input pins and limit interrupt vectors.
	// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
	// Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
	//#define LIMIT_DDR        UNDEFINED
	//#define LIMIT_PIN        UNDEFINED
	//#define LIMIT_PORT       UNDEFINED



	//#define LIMIT_INT        UNDEFINED  // Pin change interrupt enable pin
	//#define LIMIT_INT_vect   UNDEFINED
	//#define LIMIT_PCMSK      UNDEFINED // Pin change interrupt register

	// Define user-control controls (cycle start, reset, feed hold) input pins.
	// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
	//#define CONTROL_DDR       UNDEFINED
	//#define CONTROL_PIN       UNDEFINED
	//#define CONTROL_PORT      UNDEFINED




	//#define CONTROL_INT       UNDEFINED  // Pin change interrupt enable pin
	//#define CONTROL_INT_vect  UNDEFINED
	//#define CONTROL_PCMSK     UNDEFINED // Pin change interrupt register



	// Define spindle enable output pin.
	// NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
	// SPINDLE_ENABLE enables power to RC Servo (M3, M5). SPINDLE_SPEED sets Pwm of RC Servo e.g S30 S90. See arduino wiring.jpg
	//#define SPINDLE_PWM_DDR			UNDEFINED
	//#define SPINDLE_PWM_PORT		UNDEFINED


	//#define SPINDLE_ENABLE_DDR		UNDEFINED	// Note: this has no effect with PWM. Hence using Dir as Enable
	//#define SPINDLE_ENABLE_PORT		UNDEFINED


	// when USE_SPINDLE_DIR_AS_ENABLE_PIN, this is dummy
	//#define SPINDLE_DIRECTION_DDR   UNDEFINED
	//#define SPINDLE_DIRECTION_PORT  UNDEFINED


	// Variable spindle configuration below. Do not change unless you know what you are doing.
	// NOTE: Only used when variable spindle is enabled.

	//#define SPINDLE_TCCRA_REGISTER    UNDEFINED		//TCCR2A
	//#define SPINDLE_TCCRB_REGISTER    UNDEFINED		//TCCR2B
	//#define SPINDLE_OCR_REGISTER      UNDEFINED		//OCR2A
	//#define SPINDLE_COMB_BIT          UNDEFINED		//COM2A1

	// Prescaled, 8-bit Fast PWM mode.
	//#define SPINDLE_TCCRA_INIT_MASK   UNDEFINED		//((1<<WGM20) | (1<<WGM21))  // Configures fast PWM mode.
	// #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
	// #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9)
	// #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
	//#define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz (J-tech laser)
	//#define SPINDLE_TCCRB_INIT_MASK      UNDEFINED	//((1<<CS22) | (1<<CS21) | (1<<CS20))               // 1/1024 prescaler -> 61Hz (Servo Ctrl)


	// Define coolant enable output pins.
	// NOTE: COOLANT_FLOOD is used for Vacum and operated by M8, M9. COOLANT_MIST -> Bottom LED M7, M10. See Arduino Wriing.jpg
	//#define COOLANT_FLOOD_DDR		UNDEFINED
	//#define COOLANT_FLOOD_PORT		UNDEFINED
//
//
	//#define COOLANT_MIST_DDR		UNDEFINED
	//#define COOLANT_MIST_PORT		UNDEFINED
//
//
	//// Define probe switch input pin.
	//#define PROBE_DDR				UNDEFINED
	//#define PROBE_PIN				UNDEFINED
	//#define PROBE_PORT				UNDEFINED



#ifdef __cplusplus
}
#endif


#endif
