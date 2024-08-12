/*
  coolant_control.c - coolant control methods
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

#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif

void coolant_init()
{
	avr_var.COOLANT_FLOOD_DDR |= (COOLANT_FLOOD_BIT); // Configure as output pin
	pio_configure(PIOA, PIO_OUTPUT_0, avr_var.COOLANT_FLOOD_DDR, 0);

	#ifdef ENABLE_M7
		avr_var.COOLANT_MIST_DDR |= (COOLANT_MIST_BIT);
		pio_configure(PIOA, PIO_OUTPUT_0, avr_var.COOLANT_MIST_DDR, 0);
	#endif

	coolant_stop();
}


// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t coolant_get_state()
{
	uint8_t cl_state = COOLANT_STATE_DISABLE;
	uint32_t coolant_flood_pin_state = pio_get(PIOA, PIO_OUTPUT_0, COOLANT_FLOOD_BIT);

	#ifdef INVERT_COOLANT_FLOOD_PIN
		if(coolant_flood_pin_state == 0){
			cl_state |= COOLANT_STATE_FLOOD;
		};
	#else
		if(coolant_flood_pin_state > 0){
			cl_state |= COOLANT_STATE_FLOOD;
		};
	#endif


	#ifdef ENABLE_M7

		uint32_t coolant_mist_pin_state = pio_get(PIOA, PIO_OUTPUT_0, COOLANT_MIST_BIT);

		#ifdef INVERT_COOLANT_MIST_PIN
			if(coolant_mist_pin_state == 0){
				cl_state |= COOLANT_STATE_MIST;
			};
		#else
			if(coolant_mist_pin_state > 0){
				cl_state |= COOLANT_STATE_MIST;
			};
		#endif

	#endif // ENABLE_M7

  return(cl_state);
}


// Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
void coolant_stop()
{
	#ifdef INVERT_COOLANT_FLOOD_PIN
		avr_var.COOLANT_FLOOD_PORT |= (COOLANT_FLOOD_BIT);
		pio_set(PIOA, avr_var.COOLANT_FLOOD_PORT);
	#else
		avr_var.COOLANT_FLOOD_PORT &= ~(COOLANT_FLOOD_BIT);
		pio_clear(PIOA, avr_var.COOLANT_FLOOD_PORT);
	#endif

	#ifdef ENABLE_M7
		#ifdef INVERT_COOLANT_MIST_PIN
			avr_var.COOLANT_MIST_PORT |= (COOLANT_MIST_BIT);
			pio_set(PIOA, avr_var.COOLANT_MIST_PORT);
		#else
			avr_var.COOLANT_MIST_PORT &= ~(COOLANT_MIST_BIT);
			pio_clear(PIOA, avr_var.COOLANT_MIST_PORT);
		#endif
	#endif
}


// Main program only. Immediately sets flood coolant running state and also mist coolant,
// if enabled. Also sets a flag to report an update to a coolant state.
// Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser coolant_sync().
void coolant_set_state(uint8_t mode)
{
  if (sys.abort) { return; } // Block during abort.

	if (mode & COOLANT_FLOOD_ENABLE) {
		#ifdef INVERT_COOLANT_FLOOD_PIN
			avr_var.COOLANT_FLOOD_PORT &= ~(COOLANT_FLOOD_BIT);
			pio_clear(PIOA, avr_var.COOLANT_FLOOD_PORT);
		#else
			avr_var.COOLANT_FLOOD_PORT |= (COOLANT_FLOOD_BIT);
			pio_set(PIOA, avr_var.COOLANT_FLOOD_PORT);
		#endif
	}else{
		#ifdef INVERT_COOLANT_FLOOD_PIN
			avr_var.COOLANT_FLOOD_PORT |= (COOLANT_FLOOD_BIT);
			pio_set(PIOA, avr_var.COOLANT_FLOOD_PORT);
		#else
			avr_var.COOLANT_FLOOD_PORT &= ~(COOLANT_FLOOD_BIT);
			pio_clear(PIOA, avr_var.COOLANT_FLOOD_PORT);
		#endif
	}

	#ifdef ENABLE_M7
		if (mode & COOLANT_MIST_ENABLE) {
			#ifdef INVERT_COOLANT_MIST_PIN
				avr_var.COOLANT_MIST_PORT &= ~(COOLANT_MIST_BIT);
				pio_clear(PIOA, avr_var.COOLANT_MIST_PORT);
			#else
				avr_var.COOLANT_MIST_PORT |= (COOLANT_MIST_BIT);
				pio_set(PIOA, avr_var.COOLANT_MIST_PORT);
			#endif
		} else {
			#ifdef INVERT_COOLANT_MIST_PIN
				avr_var.COOLANT_MIST_PORT |= (COOLANT_MIST_BIT);
				pio_set(PIOA, avr_var.COOLANT_MIST_PORT);
			#else
				avr_var.COOLANT_MIST_PORT &= ~(COOLANT_MIST_BIT);
				pio_clear(PIOA, avr_var.COOLANT_MIST_PORT);
			#endif
		}
	#endif

  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting coolant state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void coolant_sync(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
  coolant_set_state(mode);
}


#ifdef __cplusplus
}
#endif