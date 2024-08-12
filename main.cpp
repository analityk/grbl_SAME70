#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "grbl.h"

#define IRQ_PRIOR_PIO    0
#define BLINK_PERIOD     1000

#define STRING_EOL    "\r"
#define STRING_HEADER "-- Getting Started Example --\r\n" \
		"-- " BOARD_NAME " --\r\n" \
		"-- Compiled: " __DATE__ " " __TIME__ " --" STRING_EOL




// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.

volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.




volatile bool g_b_led0_active = true;
volatile uint32_t g_ul_ms_ticks = 0;

void TC0_Handler(void) // --> timer0 in atmega
{
	volatile uint32_t isr_scr = 0;
	isr_scr = tc_get_status(TC0, 0);
	UNUSED(isr_scr);

	// compare match A (like OCR0A ISR in atmega)
	if(isr_scr & TC_SR_CPAS){
		// code ocr0a
		TIMER0_COMPA_vect();
		isr_scr = 0;
	};

	// compare match C (like overflow ISR in atmega)
	if(isr_scr & TC_SR_CPCS){
		// code timer0 overflow isr
		TIMER0_OVF_vect();
		isr_scr = 0;
	};
}

void TC3_Handler(void) // --> timer 1
{
	volatile uint32_t isr_scr = 0;
	isr_scr = tc_get_status(TC1, 0);

	UNUSED(isr_scr);

	//if(isr_scr & TC_SR_CPAS){
		//// code ocr1a
		//isr_scr = 0;
	//};

	// code ocr1a clear timer on compare but here only RC compare can triger auto reset of counter.
	// use compare on RC as ocr1a compare in atmega328P
	if(isr_scr & TC_SR_CPCS){
		// code timer 1 overflow isr
		TIMER1_COMPA_vect();
		isr_scr = 0;
	};
}

void TC6_Handler(void) // --> timer 2
{
	volatile uint32_t isr_scr = 0;
	isr_scr = tc_get_status(TC2, 0);

	UNUSED(isr_scr);

	if(isr_scr & TC_SR_CPAS){
		// code ocr1a
		isr_scr = 0;
	};

	if(isr_scr & TC_SR_CPCS){
		// code timer 1 overflow isr
		isr_scr = 0;
	};
}


void Timer1_set_freq(uint32_t freq)
{
	if(freq >= 18 && freq < 72){
		REG_TC0_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 128
		REG_TC0_RC0 = (uint32_t)(1171875 / freq);
		REG_TC0_RA0 = 0xFFFF;
		REG_TC0_RB0 = 0xFFFF;
		return;
	};

	if(freq >= 72 && freq < 500){
		REG_TC0_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 32
		REG_TC0_RC0 = (uint32_t)(4687500 / freq);
		REG_TC0_RA0 = REG_TC0_RC0 / 2;
		return;
	};

	if(freq >= 500 && freq < 20000){
		REG_TC0_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 8
		REG_TC0_RC0 = (uint32_t)(18750000 / freq);
		REG_TC0_RA0 = REG_TC0_RC0 / 2;
		return;
	};
}

void Timer2_set_freq(uint32_t freq)
{
	if(freq >= 18 && freq < 72){
		REG_TC1_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 128
		REG_TC1_RC0 = (uint32_t)(1171875 / freq);
		REG_TC1_RA0 = REG_TC1_RC0 / 2;
		return;
	};

	if(freq >= 72 && freq < 500){
		REG_TC1_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 32
		REG_TC1_RC0 = (uint32_t)(4687500 / freq);
		REG_TC1_RA0 = REG_TC1_RC0 / 2;
		return;
	};

	if(freq >= 500 && freq < 20000){
		REG_TC1_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 8
		REG_TC1_RC0 = (uint32_t)(18750000 / freq);
		REG_TC1_RA0 = REG_TC1_RC0 / 2;
		return;
	};
}

void Timer3_set_freq(uint32_t freq)
{
	if(freq >= 18 && freq < 72){
		REG_TC2_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 128
		REG_TC2_RC0 = (uint32_t)(1171875 / freq);
		//REG_TC2_RA0 = REG_TC2_RC0 / 2;
		return;
	};

	if(freq >= 72 && freq < 500){
		REG_TC2_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 32
		REG_TC2_RC0 = (uint32_t)(4687500 / freq);
		//REG_TC2_RA0 = REG_TC2_RC0 / 2;
		return;
	};

	if(freq >= 500 && freq < 20000){
		REG_TC2_CMR0 = (TC_CMR_TCCLKS_TIMER_CLOCK2 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);	// 150MHz / 8
		REG_TC2_RC0 = (uint32_t)(18750000 / freq);
		//REG_TC2_RA0 = REG_TC2_RC0 / 2;
		return;
	};
}



void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}


static void configure_console(volatile void *usart)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	sysclk_enable_peripheral_clock(ID_UART3);
	stdio_serial_init(usart, &uart_serial_options);
}

static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

static void configure_tc(void)
{
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC3);
	pmc_enable_periph_clk(ID_TC6);

	tc_init(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
	tc_init(TC1, 0, TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
	tc_init(TC2, 0, TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);

	REG_TC0_RC0 = 58593;//(uint32_t)(11718750 / freq) / 10;
	REG_TC0_RA0 = 0xFFFF; //27500;
	REG_TC0_RB0 = 0xFFFF; //27500;

	REG_TC1_RC0 = 58593;  //(uint32_t)(11718750 / freq) / 10;
	REG_TC1_RA0 = 0xFFFF; //27500;
	REG_TC1_RB0 = 0xFFFF; //

	REG_TC2_RC0 = 58593;//(uint32_t)(11718750 / freq) / 10;
	REG_TC2_RA0 = 0xFFFF; //27500;
	REG_TC2_RB0 = 0xFFFF; //

	tc_enable_interrupt(TC0, 0, TC_IER_CPCS | TC_IER_CPAS);
	tc_enable_interrupt(TC1, 0, TC_IER_CPCS);
	tc_enable_interrupt(TC2, 0, TC_IER_CPCS);

	//tc_start(TC0, 0);
	//tc_start(TC1, 0);
	tc_start(TC2, 0);
}

uint32_t volatile led = 0;

void PIOA_Handler(){
	volatile uint32_t src = REG_PIOA_ISR;



	if(src & LIMIT_MASK){
		LIMIT_INT_vect();
		if(led == 0){
			led = 1;
			pio_set(PIOC, PIO_PC8);
		}else{
			led = 0;
			pio_clear(PIOC, PIO_PC8);
		}
	};

	if(src & CONTROL_MASK){
		CONTROL_INT_vect();
	};
}

char rc = 0;
volatile uint8_t uart_transmit_ready = 0;
volatile uint8_t uart_transmit_empty = 0;
volatile uint8_t uart_receiver_ready = 0;

void UART3_Handler(){
	uint32_t src = REG_UART3_SR;
	UNUSED(src);

	// interrup for received char
	if(src & UART_SR_RXRDY){
		uart_receiver_ready = 1;
		SERIAL_RX();
	};

	// uart transmiter ready
	if(src & UART_SR_TXRDY){
		uart_transmit_ready = 1;
		SERIAL_UDRE();
	};

	if(src & UART_SR_TXEMPTY){
		uart_transmit_empty = 1;
		//SERIAL_UDRE();
	};
}

int main(void)
{

	board_init();
	wdt_disable(WDT);

	pmc_enable_periph_clk(ID_UART3);

	pio_configure(PIOD, PIO_PERIPH_A, PIO_PD30, PIO_PULLUP);	// tx
	pio_configure(PIOD, PIO_PERIPH_A, PIO_PD28, PIO_PULLUP);	// rx
	pio_configure(PIOC, PIO_OUTPUT_0, PIO_PC8, 0);				// pcb led

	configure_tc();

	configure_console(UART3);
	uart_enable_interrupt(UART3, UART_IER_RXRDY | UART_IER_TXRDY);

	Timer3_set_freq(20);

	pio_configure(PIOA, PIO_INPUT, PIO_PA18, PIO_PULLUP);




	irq_initialize_vectors();

	irq_register_handler(TC0_IRQn, 1);
	irq_register_handler(TC3_IRQn, 2);
	irq_register_handler(TC6_IRQn, 3);
	irq_register_handler(PIOA_IRQn, 4);
	irq_register_handler(UART3_IRQn, 5);
	irq_register_handler(SysTick_IRQn, 6);

	Enable_global_interrupt();


		serial_mode_normal();

		while(1){
			uint32_t pin = pio_get(PIOA, PIO_INPUT, PIO_PA18);
			printf("%ul\r\n", pin);
			delay_ms(333);
		}

	// main grbl.c


		settings_restore(0xFF);

	  settings_init(); // Load Grbl settings from EEPROM

	  stepper_init();  // Configure stepper pins and interrupt timers
	  system_init();   // Configure pinout pins and pin-change interrupt

	  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

	  //sei(); // Enable interrupts

	  // Initialize system state.
	  #ifdef FORCE_INITIALIZATION_ALARM
	  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
		sys.state = STATE_ALARM;
	  #else
		sys.state = STATE_IDLE;
	  #endif

	  // Check for power-up and set system alarm if homing is enabled to force homing cycle
	  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
	  // startup scripts, but allows access to settings and internal commands. Only a homing
	  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
	  // NOTE: The startup script will run after successful completion of the homing cycle, but
	  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
	  // things uncontrollably. Very bad.

	  //#ifdef HOMING_INIT_LOCK
	  //if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
	  //#endif

	  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
	  // will return to this loop to be cleanly re-initialized.
	  for(;;) {

		  // Reset system variables.
		  uint8_t prior_state = sys.state;
		  memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
		  sys.state = prior_state;
		  sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
		  sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
		  sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
		  memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
		  sys_probe_state = 0;
		  sys_rt_exec_state = 0;
		  sys_rt_exec_alarm = 0;
		  sys_rt_exec_motion_override = 0;
		  sys_rt_exec_accessory_override = 0;

		  // Reset Grbl primary systems.
		  serial_reset_read_buffer(); // Clear serial read buffer
		  gc_init(); // Set g-code parser to default state
		  spindle_init();
		  coolant_init();
		  limits_init();
		  probe_init();
		  plan_reset(); // Clear block buffer and planner variables
		  st_reset(); // Clear stepper subsystem variables.

		  // Sync cleared gcode and planner positions to current system position.
		  plan_sync_position();
		  gc_sync_position();

		//// Reset system variables.
		//sys.abort = false;
		//sys.execute = 0;
		//if (bit_istrue(settings.flags,BITFLAG_AUTO_START)) { sys.auto_start = true; }
		//else { sys.auto_start = false; }

		// Start Grbl main loop. Processes program inputs and executes them.
		protocol_main_loop();

	  }


	while(1){
		while(!g_b_led0_active);

		if(g_b_led0_active){
			printf("1 ");
		}


		mdelay(500);
	}
}

