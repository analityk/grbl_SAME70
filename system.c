/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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


void system_init()
{
  avr_var.CONTROL_DDR &= ~(CONTROL_MASK); // Configure as input pins
  pmc_enable_periph_clk(ID_PIOA);
  pio_configure(PIOA, PIO_INPUT, CONTROL_MASK, PIO_PULLUP);

  #ifdef DISABLE_CONTROL_PIN_PULL_UP
    //avr_var.CONTROL_PORT &= ~(CONTROL_MASK); // Normal low operation. Requires external pull-down.
	//pio_pull_down(PIOA, avr_var.CONTROL_PORT, avr_var.CONTROL_PORT);
	serial_mode_normal();
	printf("system init\r\n");
	serial_mode_grbl();
  #else
  	serial_mode_normal();
  	printf("all GPIO are pull-up'ed\r\n");
  	serial_mode_grbl();
    avr_var.CONTROL_PORT |= CONTROL_MASK;   // Enable internal pull-up resistors. Normal high operation.
	//pio_pull_up(PIOA, avr_var.CONTROL_PORT, avr_var.CONTROL_PORT);
  #endif
  //avr_var.CONTROL_PCMSK |= CONTROL_MASK;  // Enable specific pins of the Pin Change Interrupt
  cfg_interrupt_pioa(CONTROL_MASK, 15);
  // all input gpios where interrupt is generate are configure in main file

  //PCICR |= (1 << CONTROL_INT);   // Enable Pin Change Interrupt
}


// Returns control pin state as a uint8 bitfield. Each bit indicates the input pin state, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Bitfield organization is
// defined by the CONTROL_PIN_INDEX in the header file.
uint32_t system_control_get_state()
{
  uint32_t control_state = 0;

  uint32_t pin =  pio_get(PIOA, PIO_INPUT, CONTROL_MASK); //(avr_var.CONTROL_PIN & CONTROL_MASK) ^ CONTROL_MASK;

  serial_mode_normal();
  printf("system control get state %u \r\n", pin);
  serial_mode_grbl();

  //#ifdef INVERT_CONTROL_PIN_MASK
    //pin ^= INVERT_CONTROL_PIN_MASK;
  //#endif

  if(pin){
    #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
	  if(io_bit_is_true(PIOA, CONTROL_SAFETY_DOOR_BIT)){
		  control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR;
		    serial_mode_normal();
		    printf("CONTROL_PIN_INDEX_SAFETY_DOOR \r\n");
		    serial_mode_grbl();
	  }
    #else
	  if(io_bit_is_true(PIOA, CONTROL_FEED_HOLD_BIT)){
		  control_state |= CONTROL_PIN_INDEX_FEED_HOLD;
		    serial_mode_normal();
		    printf("CONTROL_PIN_INDEX_FEED_HOLD \r\n");
		    serial_mode_grbl();
	  }
    #endif
	if(io_bit_is_true(PIOA, CONTROL_RESET_BIT)){
		control_state |= CONTROL_PIN_INDEX_RESET;
		  serial_mode_normal();
		  printf("CONTROL_PIN_INDEX_RESET \r\n");
		  serial_mode_grbl();
	}
	if(io_bit_is_true(PIOA, CONTROL_CYCLE_START_BIT)){
		control_state |= CONTROL_PIN_INDEX_CYCLE_START;
		  serial_mode_normal();
		  printf("CONTROL_PIN_INDEX_CYCLE_START\r\n");
		  serial_mode_grbl();
	}
  }
  return(control_state);
}


// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
//ISR(CONTROL_INT_vect)

void CONTROL_INT_vect(void)
{
	serial_mode_normal();
	printf("CONTROL_INT_vect\r\n");
	serial_mode_grbl();

  uint32_t pin = system_control_get_state();
  if(pin){

    if(bit_istrue(pin,CONTROL_PIN_INDEX_RESET)){
      mc_reset();
    }

    if(bit_istrue(pin,CONTROL_PIN_INDEX_CYCLE_START)){
      bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
    }

    #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN

      if(bit_istrue(pin,CONTROL_PIN_INDEX_FEED_HOLD)){
        bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
	  }

    #else

      if(bit_istrue(pin,CONTROL_PIN_INDEX_SAFETY_DOOR)){
        bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
	  }

    #endif
  }
}


// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint32_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
  #else
    return(false); // Input pin not enabled, so just return that it's closed.
  #endif
}


// Executes user startup script, if stored.
void system_execute_startup(char *line)
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++){
    if(!(settings_read_startup_line(n, line))){
      line[0] = 0;
      report_execute_startup_message(line,STATUS_SETTING_READ_FAIL);
	  		serial_mode_normal();
	  		printf("STATUS_SETTING_READ_FAIL\r\n");
	  		serial_mode_grbl();
    } else {
      if(line[0] != 0){
        uint8_t status_code = gc_execute_line(line);
        report_execute_startup_message(line,status_code);
			serial_mode_normal();
			printf("STATUS_SETTING_READ ok\r\n");
			serial_mode_grbl();
      }
    }
  }
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.

uint32_t system_execute_line(char *line)
{
	serial_mode_normal();
	printf("system_execute_line\r\n");
	serial_mode_grbl();
  uint8_t char_counter = 1;
  uint8_t helper_var = 0; // Helper variable
  float parameter, value;
  switch( line[char_counter] ){
    case 0 : report_grbl_help(); break;
    case 'J' : // Jogging
      // Execute only if in IDLE or JOG states.
      if(sys.state != STATE_IDLE && sys.state != STATE_JOG){ return(STATUS_IDLE_ERROR); }
      if(line[2] != '='){ return(STATUS_INVALID_STATEMENT); }
      return(gc_execute_line(line)); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
      break;
    case '$': case 'G': case 'C': case 'X':
      if( line[2] != 0 ){ return(STATUS_INVALID_STATEMENT); }
      switch( line[1] ){
        case '$' : // Prints Grbl settings
          if( sys.state & (STATE_CYCLE | STATE_HOLD) ){ return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
          else { report_grbl_settings(); }
          break;
        case 'G' : // Prints gcode parser state
          // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
          report_gcode_modes();
          break;
        case 'C' : // Set check g-code mode [IDLE/CHECK]
          // Perform reset when toggling off. Check g-code mode should only work if Grbl
          // is idle and ready, regardless of alarm locks. This is mainly to keep things
          // simple and consistent.
          if( sys.state == STATE_CHECK_MODE ){
            mc_reset();
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if(sys.state){ return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break;
        case 'X' : // Disable alarm lock [ALARM]
          if(sys.state == STATE_ALARM){
            // Block if safety door is ajar.
            if(system_check_safety_door_ajar()){ return(STATUS_CHECK_DOOR); }
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Don't run startup script. Prevents stored moves in startup from causing accidents.
          } // Otherwise, no effect.
          break;
      }
      break;
    default :
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ){ return(STATUS_IDLE_ERROR); }
      switch( line[1] ){
        case '#' : // Print Grbl NGC parameters
          if( line[2] != 0 ){ return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;
        case 'H' : // Perform homing cycle [IDLE/ALARM]
#ifdef VISUAL_HOMING
			if(line[2] == 0)
			{
				settings.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
				// Setting Machine axes to an offset. There is no G or M code to do this in Grbl. Hence this customisation
				// NOTE: settings.max_travel[] is stored as a negative value.
				sys_position[0] = (settings.max_travel[0] + settings.homing_feed_rate) * settings.steps_per_mm[0];	// Leaching in $24, X
				sys_position[1] = (settings.max_travel[1] + settings.homing_seek_rate) * settings.steps_per_mm[1];	// $25, Y
				sys_position[2] = (settings.max_travel[2] + settings.homing_pulloff) * settings.steps_per_mm[2];	// $27, Z
				sys_position[3] = 0;	// C is endless

				// Sync gcode parser and planner positions to homed position.
				gc_sync_position();
				plan_sync_position();

			}		// now our machine will not crash the nozzle
#else
			if(bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE)){ return(STATUS_SETTING_DISABLED); }
			if(system_check_safety_door_ajar()){ return(STATUS_CHECK_DOOR); } // Block if safety door is ajar.
			sys.state = STATE_HOMING; // Set system state variable
			if(line[2] == 0){
				mc_homing_cycle(HOMING_CYCLE_ALL);
#ifdef HOMING_SINGLE_AXIS_COMMANDS
			}
			else if(line[3] == 0){
				switch (line[2]){
				case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
				case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
				case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
				default: return(STATUS_INVALID_STATEMENT);
				}
#endif
			}
			else { return(STATUS_INVALID_STATEMENT); }
			if(!sys.abort){  // Execute startup scripts after successful homing.
				sys.state = STATE_IDLE; // Set to IDLE when complete.
				st_go_idle(); // Set steppers to the settings idle state before returning.
				if(line[2] == 0){ system_execute_startup(line); }
			}
#endif // VISUAL_HOMING

          break;
        case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
          if((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)){ return(STATUS_INVALID_STATEMENT); }
          system_set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
          break;
        case 'I' : // Print or store build info. [IDLE/ALARM]
          if( line[++char_counter] == 0 ){
            settings_read_build_info(line);
            report_build_info(line);
          #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
            } else { // Store startup line [IDLE/ALARM]
              if(line[char_counter++] != '='){ return(STATUS_INVALID_STATEMENT); }
              helper_var = char_counter; // Set helper variable as counter to start of user info line.
              do {
                line[char_counter-helper_var] = line[char_counter];
              } while (line[char_counter++] != 0);
              settings_store_build_info(line);
          #endif
          }
          break;
        case 'R' : // Restore defaults [IDLE/ALARM]
          if((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)){ return(STATUS_INVALID_STATEMENT); }
          switch (line[5]){
            #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
              case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
              case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            #endif
            #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
              case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            #endif
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Force reset to ensure settings are initialized correctly.
          break;
        case 'N' : // Startup lines. [IDLE/ALARM]
          if( line[++char_counter] == 0 ){ // Print startup lines
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++){
              if(!(settings_read_startup_line(helper_var, line))){
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
            if(sys.state != STATE_IDLE){ return(STATUS_IDLE_ERROR); } // Store only when idle.
            helper_var = true;  // Set helper_var to flag storing method.
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)){ return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '='){ return(STATUS_INVALID_STATEMENT); }
          if(helper_var){ // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line); // Set helper_var to returned status code.
            if(helper_var){ return(helper_var); }
            else {
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)){ return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)){ return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}



void system_flag_wco_change()
{
  #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    protocol_buffer_synchronize();
  #endif
  sys.report_wco_counter = 0;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if(idx==X_AXIS){
      pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if(idx==Y_AXIS){
      pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    pos = steps[idx]/settings.steps_per_mm[idx];
  #endif
  return(pos);
}


void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++){
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}


// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif


// Checks and reports if target array exceeds machine travel limits.
uint8_t system_check_travel_limits(float *target)
{
  uint8_t idx;

//#ifdef VISUAL_HOMING
  for (idx = 0; idx < N_AXIS-1; idx++){		// C Axis is endless


//#ifdef HOMING_FORCE_SET_ORIGIN
      //// When homing forced set origin is enabled, soft limits checks need to account for directionality.
      //// NOTE: max_travel is stored as negative
      //if(bit_istrue(settings.homing_dir_mask,bit(idx))){
        //if(target[idx] < 0 || target[idx] > -settings.max_travel[idx]){ return(true); }
      //} else {
        //if(target[idx] > 0 || target[idx] < settings.max_travel[idx]){ return(true); }
      //}
//#else
      // NOTE: max_travel is stored as negative
      if(target[idx] > 0 || target[idx] < settings.max_travel[idx]){ return(true); }
//#endif
  }
  return(false);
}


// Special handlers for setting and clearing Grbl's real-time execution flags.
void system_set_exec_state_flag(uint8_t mask){
	irqflags_t flags;
	flags = cpu_irq_save();
	sys_rt_exec_state |= (mask);
	__DSB();
	__ISB();
	cpu_irq_restore(flags);
}

void system_clear_exec_state_flag(uint8_t mask){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_state &= ~(mask);
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_set_exec_alarm(uint8_t code){
  irqflags_t flags;
  flags = cpu_irq_save();

  sys_rt_exec_alarm = code;

  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_clear_exec_alarm(){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_alarm = 0;
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_set_exec_motion_override_flag(uint8_t mask){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_motion_override |= (mask);
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_set_exec_accessory_override_flag(uint8_t mask){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_accessory_override |= (mask);
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_clear_exec_motion_overrides(){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_motion_override = 0;
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

void system_clear_exec_accessory_overrides(){
  irqflags_t flags;
  flags = cpu_irq_save();
  sys_rt_exec_accessory_override = 0;
  __DSB();
  __ISB();
  cpu_irq_restore(flags);
}

#ifdef __cplusplus
}
#endif
