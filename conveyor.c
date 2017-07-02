/*

AUTHORS:
Ben Wolfe | V00205547
Avery Bigrigg | V00770041

DATE:
11.04.2017

DESCRIPTION:
This firmware controls a DC-motor controlled conveyor system which sorts black, 
white, steel, and aluminum blocks onto a tray which is controlled by a stepper 
motor.  The system is controlled using two optical sensors and a reflective 
sensor.  External interrupts trigger task execution.

*/

//******************************************************************************
//******************************************************************** LIBRARIES																	   
//******************************************************************************

#include <avr/io.h> //AVR input/output library
#include <avr/interrupt.h> //AVR interrupt library
#include <stdlib.h> //standard C library
#include <stdint.h> //integer type C library
#include <math.h> //math library (sqrt root function)

//******************************************************************************
//**************************************************** CONSTANTS AND DEFINITIONS
//******************************************************************************

//--------------------------------------------------------  BLOCK CLASSIFICATION
const int ALUMINUM_MIN = 0; const int ALUMINUM_MAX = 101; 
const int STEEL_MIN = 101; const int STEEL_MAX = 701; 
const int WHITE_MIN = 701; const int WHITE_MAX = 886; 
const int BLACK_MIN = 886; const int BLACK_MAX = 1022; 

//---------------------------------------------------------------- STEPPER MOTOR
#define T1_FREQ 1000000 //frequency of timer 1 with prescaling
#define SPR 200 //steps per revolution

//refer to 'Atmel Stepper Optimization' for these equations
#define ALPHA (2*3.14159/SPR)
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) 
#define A_SQ (long)(ALPHA*2*10000000000)         
#define A_x20000 (int)(ALPHA*20000)

//profile states
#define CW 0
#define CCW 1

#define STOP 0
#define ACCEL 1
#define DECEL 2
#define RUN 3

int MAX_SPEED = 0; //max: 1120 (2.8ms/step)
const int ACCELERATION = 7000; 
const int DECELERATION = 7000; 

//stepper positing tracking
const int STEEL = 1;
const int WHITE = 2;
const int ALUMINUM = 3;
const int BLACK = 4;

//--------------------------------------------------------------------- DC MOTOR
const unsigned int DC_FORWARD = 0b00000001;
const unsigned int DC_REVERSE = 0b00000100;
const unsigned int DC_HAULT = 0b00000000;

//******************************************************************************
//************************************************************* GLOBAL VARIABLES
//******************************************************************************

//----------------------------------------------------------------------- BLOCKS
typedef struct block{
	
	int material;
	int match;
	struct block* next;
	
} block;

block *head = NULL;
block *tail = NULL;

int dropped_steel_count = 0; 
int dropped_black_count = 0; 
int dropped_aluminum_count = 0;
int dropped_white_count = 0; 
int total_dropped_weight = 0;

int total_conveyor_count = 0;

//---------------------------------------------------------------- STEPPER MOTOR
typedef struct {
	
	unsigned char run_state : 3; //RUN, STOP, ACCEL, or DECEL
	unsigned char direction : 1; 	     //CW or CCW rotation

	signed int min_delay;        //delay corresponding to max speed
	unsigned int step_delay; 	 //period of next timer delay

	unsigned int decel_start;    //step number to start deceleration
	signed int number_of_decel_steps;        //steps allocated for deceleration
	
	signed int accel_count;		 //step tracking for accel and decel

} speed_ramp_data;

speed_ramp_data stepper_speed_data;

unsigned int steps[4] = {0b11011000, 0b10111000, 0b10110100, 0b11010100};
int step_position = 0; //tracks the position in the step array

unsigned int step_count = 0; //counts steps while moving

int current_position = 0; //tracks the current position of the sorting tray
int current_direction = 0; //tracks the direction that the stepper moved last

int required_position = 0; //tracks the desired next position
int required_rotation = 0; //tracks number of steps required to get there

int stepper_movement_status = 0; //tracks whether the stepper is busy or not
							   //0: free, 1: active
							   
//--------------------------------------------------------------- ADC CONVERSION
volatile int temp_lower_bits = 0; //temporarily store ADC conversion result (bits 0-7)
volatile int temp_upper_bits = 0; //temporarily store ADC conversion result (bits 8-9)
volatile int temp_reflective_value = 0; //temporarily store 16-bit conversion

volatile int lower_bits = 0; //reflective sensor bits 0-7
volatile int upper_bits = 0; //reflective sensor bits 8-9
volatile int reflective_value = 1023; //16 bit reflective value

//---------------------------------------------------------------------- SENSORS
//EXIT SENSOR
int part_waiting = 0;
int exit_interrupt_count = 0;

//ADC SENSOR
int ADC_conversion_count = 0;

//PAUSE BUTTON
int pause_flag = 0;
int pause_press_count = 0;

//EXIT SENSOR
int ramp_down_system_flag = 0;
int ramp_down_timer_interrupt_count = 0;

//******************************************************************************
//******************************************************** FUNCTION DECLARATIONS
//******************************************************************************

//----------------------------------------------------------------- MAIN SORTING
void main_sorting_routine();
void check_pause_status();
void pause_system();
void shut_down_system();

//------------------------------------------------- GENERAL SYSTEM CONFIGURATION
void configure_system();
void calibrate_reflective_sensor();
void delay(int msecTimeDelay);
void blink_LEDs();

//---------------------------------------------------------------- STEPPER MOTOR
void home_stepper();
void rotate_stepper();
void calculate_stepper_profile(signed int step, unsigned int accel, unsigned int decel, unsigned int speed);
void send_stepper_pulse(signed char dir);
void speed_and_position_test();

//--------------------------------------------------------- BLOCK CLASSIFICATION
void classify_block();
void create_new_block(block **newBlock);
void add_to_list(block **newBlock);
void drop_part();

//******************************************************************************
//***************************************************************** MAIN ROUTINE
//******************************************************************************

int main(void){
	
	configure_system();
	home_stepper();
	//calibrate_reflective_sensor();
	//speed_and_position_test(); //tests the stepper motor
	main_sorting_routine();
	shut_down_system();
	
	return(0);	
} //end main

//******************************************************************************
//********************************************************* FUNCTION DEFINITIONS 
//******************************************************************************

//----------------------------------------------------------------- MAIN SORTING
void main_sorting_routine(){
	
	OCR0A = 85; //set PWM duty cycle
	PORTB = DC_FORWARD;
	
	while(1){ //main loop (empty list loop)
				
		check_pause_status(); //check the status of pause button
		if(ramp_down_system_flag == 1) break; //check status of ramp down button
		
		//parts in list loop		
		while(total_conveyor_count > 0){ 
			
			check_pause_status(); //check the status of pause button
			if(ramp_down_system_flag == 1) break; //check status of ramp down button
			
			required_position = head->material;
			
			if((current_position != required_position) && (stepper_movement_status == 0) && (part_waiting == 0)){
				rotate_stepper();
			}
			
			else if((current_position != required_position) && (stepper_movement_status == 0) && (part_waiting == 1)){
				rotate_stepper();
			}
			
			else if((current_position == required_position) && (part_waiting == 1)){
				
				part_waiting = 0;
				PORTB = DC_FORWARD;
				
				drop_part();				
				delay(150);

			}
			
		} //end parts in list while
	} //end empty list while

	return;
}

void check_pause_status(){
	
	if(pause_flag == 1){
			
		while(stepper_movement_status == 1){
			PORTC = 0xff;
		}
		PORTC = 0x00;
		
		pause_system();
			
	}
	
	return;
}

void pause_system(){
	
	while(pause_press_count < 6){
		
		if(pause_press_count == 1) PORTC = 0b00010000 | dropped_steel_count;
		else if(pause_press_count == 2) PORTC = 0b00100000 | dropped_white_count;
		else if(pause_press_count == 3) PORTC = 0b01000000 | dropped_aluminum_count;
		else if(pause_press_count == 4) PORTC = 0b10000000 | dropped_black_count;
		
		else if(pause_press_count == 5) PORTC = 0b11110000 | total_conveyor_count;	
		
	} //end while
	
	pause_flag = 0;
	pause_press_count = 0;
	PORTC = 0x00;
	
	//restart belt
	PORTB = DC_FORWARD;
	
	return;
} //end pause_system

void shut_down_system(){
	
	PORTA = 0x00;
	PORTB = DC_HAULT;
	
	cli(); //globally disable interrupts
	
	while(1){
		
		PORTC = 0b00010000 | dropped_steel_count;
		delay(1000);
		PORTC = 0b00100000 | dropped_white_count;
		delay(1000);
		PORTC = 0b01000000 | dropped_aluminum_count;
		delay(1000);
		PORTC = 0b10000000 | dropped_black_count;
		delay(1000);

	}
	
	return;
}

//------------------------------------------------- GENERAL SYSTEM CONFIGURATION
void configure_system(){

	cli(); //globally disable interrupts

	//configure the clock frequency
	CLKPR = (1 << CLKPCE); //clock prescaler change enable
	CLKPR = 0; //set all bits to 0 for 8MHz
	
	//configure inputs and outputs
	DDRA = 0xff; //all pins on port A to output (LED bits 8-9 and stepper)
	DDRB = 0xff; //all pins on port B to output to drive DC motor
	DDRC = 0xff; //all pins on port C to output (LED bits 0-7)
	DDRD = 0x00; //all pins on port D to input to monitor interrupts (redundant)
	DDRE = 0x00; //all pins on port E to input to monitor interrupts (redundant)

	//enable external interrupts
	EIMSK |= (1<<INT0); //enable exit sensor interrupt (INT0 - PD0)
	EICRA |= (1<<ISC01); //configure to falling edge interrupt

	EIMSK |= (1<<INT1); //enable reflective-optical sensor interrupt (INT1 - PD1)
	EICRA |= (1<<ISC11) | (1<<ISC10); //rising edge interrupt
	
	EIMSK |= (1<<INT5); //enable 'Pause System' interrupt (INT5 - PE5)
	EICRB |= (1<<ISC51) | (1<<ISC50); //configure to rising edge interrupt
	
	EIMSK |= (1<<INT6); //enable 'System Shut-down' interrupt (INT6 - PE6)
	EICRB |= (1<<ISC61) | (1<<ISC60); //configure to rising edge interrupt

	//configure 8-bit timer 0 for PWM
	TCCR0A |= (1<<WGM01) | (1<<WGM00); //set timer to Fast PWM Mode
	TCCR0A |= (1<<COM0A1); //clear timer on compare match
	TCCR0B |= (1<<CS01) | (1<<CS00); //set clock prescaler to 64
	
	//configure 8-bit timer2 with interrupt for system ramp down
	TCCR2A |= (1<<WGM21); //set timer to CTC mode
	TIMSK2 |= (1<<OCIE2A); //enable timer/counter 2 output compare interrupt

	//configure 16-bit timer 1 with timer interrupt for stepper motor
	TCCR1B = (1<<WGM12); //set timer/counter 1 to CTC mode
	TIMSK1 = (1<<OCIE1A); //enable timer/counter 1 output compare interrupt

	//configure ADC
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable ADC and ADC interrupt with 64 prescaling
	ADMUX |= (1<<REFS0) | (1<<MUX0); //set the reference voltage to Vcc and ADC to PF1
									 //only PF1 and PF2 are 'free pins'
	
	//configure 16-bit timer 3 without interrupt for delay()
	TCCR3B |= (1<<WGM32) | (1<<CS31); //configure CTC mode with factor 8 prescaling
	TIFR3 |= (1<<OCF3A); //enable output compare match flag
	
	stepper_speed_data.run_state = STOP; //initialize stepper movement state	

	sei(); //globally enable interrupts

	delay(500);
	blink_LEDs();
	
	return;	
}

void calibrate_reflective_sensor(){
	
	OCR0A = 85; //set PWM duty cycle
	PORTB = DC_FORWARD;
	
	while(1){
		
		//PORTC = ADC_conversion_count;
		
	}
	
	return;
}

/* count the prescribed number of milliseconds before returning where
	delay() was called */
void delay(int msecTimeDelay){
	
	unsigned int counter = 0; //variable for counting the loop number
	
	OCR3A = 1000; //set output compare register upper limit (1000)
	TCNT3 = 0; //initialize the timer to 0

	while (counter < msecTimeDelay){
		if((TIFR3 &0x02) == 0x02){ // Check for interrupt flag

			TIFR3 |= (1<<OCF3A); //manually reset the timer interrupt flag
			counter++; // Increment the loop counter

		} //end If
	} //end while

	return;
}

void blink_LEDs(){
	
	PORTC=0xff; delay(75);
	PORTC=0x00; delay(75);
	PORTC=0xff; delay(75);
	PORTC=0x00; delay(75);
	
	return;

}

//---------------------------------------------------------------- STEPPER MOTOR

/* home_stepper() rotates the motor until the Hall effect sensor picks up magnet.
   This will occur when 'black' is inline with the conveyor. */
void home_stepper(){

	while((PINE &0b00010000) != 0b00000000){
		step_position++;
		if(step_position > 3) step_position=0;

		PORTA = steps[step_position];
		delay(15); //delay 10ms between steps
	}

	/* An extra loop is requied incase if the magnet starts close enough for the 
		Hall-Effect sensor to pick it up but a few steps away from true home */
	for(int offSetFromHome = 0; offSetFromHome <= 100; offSetFromHome++){
		step_position++;
		if(step_position > 3) step_position=0;

		PORTA = steps[step_position];
		delay(15); //delay 10ms between steps
	}
	
	while((PINE &0b00010000) != 0b00000000){
		step_position++;
		if(step_position > 3) step_position=0;

		PORTA = steps[step_position];
		delay(15); //delay 10ms between steps
	}

	current_position = BLACK;	
	current_direction = CW;
	
	blink_LEDs();
	
	return;
}

/* rotate_stepper() calculates the required steps and calls configureStepperProfile()
   to generate the profile.  This profile is then implemented by a series of
   timer interrupts.  As the interrupts fire, pulses are sent to the stepper
   motor.  It's easier this way, since it only means calling one function. */
void rotate_stepper(){
	
	//negative value indicates CCW rotation; clockwise value indicates CW rotation
	required_rotation = current_position-required_position;
	
	if(required_rotation == 1) required_rotation = 50;
	else if (required_rotation == -1) required_rotation = -50;

	else if(required_rotation == 2 || required_rotation == -2) {

		if(current_direction == CW){
			required_rotation = 100;
			
			} else if(current_direction == CCW){
			required_rotation = -100;
		}
	} //CW or CCW momentum check
	
	else if(required_rotation == -3) required_rotation = 50;
	else if(required_rotation == 3) required_rotation = -50;

	else return; //the only other case is required_rotation == 0 which means the
				 //stepper doesn't need to move
	
	if(total_dropped_weight < 216) MAX_SPEED = 750;
	else if(total_dropped_weight >= 216 && total_dropped_weight < 432) MAX_SPEED = 600;
	else if(total_dropped_weight >= 432) MAX_SPEED = 500;

	calculate_stepper_profile(required_rotation, ACCELERATION, DECELERATION, MAX_SPEED);
	
	return;
}

/* calculate_stepper_profile() sets the direction, the minimum delay corresponding 
   to maximum speed, steps required for acceleration, and steps required for
   deceleration.  Checks are performed to catch boundary conditions.  The first
   time delay is calculated here and the timer is started. */
void calculate_stepper_profile(signed int step, unsigned int accel, unsigned int decel, unsigned int speed){
	
	stepper_movement_status = 1; //flag to know when stepper is busy
	
	unsigned int maxSpeedStepLimit; //number of steps before hitting max speed
	unsigned int accelLimit; //number of steps before starting deceleration
	
	//set the direction of motion
	if(step < 0){
		stepper_speed_data.direction = CCW;
		current_direction = CCW;
		step = -step;
	} else {
		current_direction = CW;
		stepper_speed_data.direction = CW;
	}
	
	//calculate the minimum time delay corresponding to max speed
	stepper_speed_data.min_delay = A_T_x100 / speed; 
	
	//calculate first, c0, step delay
	stepper_speed_data.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100; 
	
	//steps to reach max speed
	maxSpeedStepLimit = (long)speed*speed/(long)(((long)A_x20000*accel)/100); 
	
	//in practice, need at least 1 step to get any speed at all
	if(maxSpeedStepLimit == 0){
		maxSpeedStepLimit = 1; 
	}

	//steps to reach deceleration time
	accelLimit = ((long)step*decel) / (accel+decel); 
	
	//accelerated at least 1 step before starting deceleration
	if(accelLimit == 0){
		accelLimit = 1; 
	}

	//check to see if the stepper can reach full speed before deceleration
	if(accelLimit <= maxSpeedStepLimit){
		
		//in this case the stepper can't reach full speed
		stepper_speed_data.number_of_decel_steps = accelLimit - step; 
		
	} else {
		
		//in this case, the stepper reaches full speed
		stepper_speed_data.number_of_decel_steps = -((long)maxSpeedStepLimit*accel)/decel; 
	}

	//decelerated at least 1 step
	if(stepper_speed_data.number_of_decel_steps == 0){
		stepper_speed_data.number_of_decel_steps = -1; 
	}

	//find the step where we need to start deceleration
	stepper_speed_data.decel_start = step + stepper_speed_data.number_of_decel_steps; 

	//check if the speed is low enough to go straight to run state
	if(stepper_speed_data.step_delay <= stepper_speed_data.min_delay){
		stepper_speed_data.step_delay = stepper_speed_data.min_delay;
		stepper_speed_data.run_state = RUN;
	} else {
		stepper_speed_data.run_state = ACCEL;
	}

	stepper_speed_data.accel_count = 0; //reset the acceleration/deceleration counter
	OCR1A = 10; //give a buffer before firing the interrupt for the first time
	
	//start the timer with 8 as a prescaling factor
	TCCR1B |= ((0<<CS12)|(1<<CS11)|(0<<CS10)); 
	
	return;
}

void send_stepper_pulse(signed char dir){
//send_stepper_pulse() sends a step to the stepper motor when the
//timer1 interrupt fires.
	
	if(dir == CCW){
		step_position--;
		if(step_position < 0) step_position = 3; //track array bounds
		
	} else {
		step_position++;
		if(step_position > 3) step_position = 0; //track array bounds
	}
	
	//output the step
	PORTA = steps[step_position];
	
	return;
} //end send_stepper_pulse

void speed_and_position_test(){
//speed_and_position_test() runs the stepper through a series of movements designed 
//to test all the cases specified in rotate_stepper().

	required_position = STEEL;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff; //leave lights on to indicate the stepper is 'active' and
					  //can't take new position commands
	}
	
	PORTC = 0x00; //turn off lights when the stepper reaches its desired position
	delay(25);
	
	required_position = WHITE;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
	
	required_position = ALUMINUM;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
		
	required_position = BLACK;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
	
	required_position = WHITE;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);

	required_position = BLACK;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
	
	required_position = ALUMINUM;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);

	required_position = WHITE;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);

	required_position = STEEL;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
	
	required_position = BLACK;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
	
	required_position = WHITE;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);

	required_position = BLACK;
	if(stepper_movement_status == 0) rotate_stepper();
	
	while(stepper_movement_status == 1){
		PORTC = 0xff;
	}
	
	PORTC = 0x00;
	delay(25);
}

//--------------------------------------------------------- BLOCK CLASSIFICATION

/* classify_block() adds a block to the list using the reflective value obtained
	from the ADC conversion */
void classify_block(){
	
	if((reflective_value >= ALUMINUM_MIN) && (reflective_value < ALUMINUM_MAX)){

			block *newBlock;
			create_new_block(&newBlock);
			
			newBlock->material = ALUMINUM; //material number assignment
			
			add_to_list(&newBlock); //add the block to the list
			
			total_conveyor_count++;
			
	} else if ((reflective_value >= STEEL_MIN) && (reflective_value < STEEL_MAX)){

			block *newBlock;
			create_new_block(&newBlock);

			newBlock->material = STEEL; //material number assignment
			
			add_to_list(&newBlock);
			
			total_conveyor_count++;
			
	} else if ((reflective_value >= WHITE_MIN) && (reflective_value < WHITE_MAX)){

			block *newBlock;
			create_new_block(&newBlock);

			newBlock->material = WHITE; //material number assignment
			
			add_to_list(&newBlock);
			
			total_conveyor_count++;
						
	} else if ((reflective_value >= BLACK_MIN) && (reflective_value <= BLACK_MAX)){

			block *newBlock;
			create_new_block(&newBlock);

			newBlock->material = BLACK; //material number assignment
			
			add_to_list(&newBlock);
			
			total_conveyor_count++;
			
	} else {

		//blink lights and halt belt to indicate a classification failure
		PORTB = DC_HAULT;
		
		PORTC=0xff; delay(100);
		PORTC=0x00; delay(100);
		PORTC=0xff; delay(100);
		PORTC=0x00; delay(500);
		return;
	} 
	
	return;
} //end classify_block

/* create_new_block() allocates the memory for a new block */
void create_new_block(block **newBlock){
	
	*newBlock = malloc(sizeof(block));
	(*newBlock)->next = NULL;
	
	return;
} //end create_new_block

/* add_to_list() adds new element in the linekd list */
void add_to_list(block **newBlock){
	
	if(tail != NULL){ //not an empty queue
		
		tail->next = *newBlock;
		tail = *newBlock;
		
	} else { //empty queue
		
		head = *newBlock;
		tail = *newBlock;
		
	}

	return;
} //end add_to_list

/* partDrop() removes a block from the list */
void drop_part(){
	
	if(head == NULL) total_conveyor_count = 0;

	else if(head != NULL){

		block *droppedPart = head;

		if(head->next != NULL){
			head = head->next;
		} else {
			head = NULL;
			tail = NULL;
		}

		if(droppedPart->material == STEEL){
			dropped_steel_count++;
			total_dropped_weight += 31;
			
			total_conveyor_count--;
		}

		else if(droppedPart->material == BLACK){
			dropped_black_count++;
			total_dropped_weight += 6;
			
			total_conveyor_count--;
		}

		else if(droppedPart->material == ALUMINUM){
			dropped_aluminum_count++;
			total_dropped_weight += 11;
			
			total_conveyor_count--;
		}
		
		else if(droppedPart->material == WHITE){
			dropped_white_count++;
			total_dropped_weight += 6;
	
			total_conveyor_count--;
		} 
		
		free(droppedPart);
	} //end else if

	return;
} //end drop part

//******************************************************************************
//*********************************************************** INTERRUPT ROUTINES
//******************************************************************************

//exit sensor interrupt
ISR(INT0_vect){
	
	required_position = head->material;

	if(current_position != required_position){
		PORTB = DC_HAULT;
		part_waiting = 1;
	} else {
		part_waiting = 1;
	}
			
} //end ISR0

//reflective optical sensor interrupt
ISR(INT1_vect){

	reflective_value = 1023; //reset the reference reflective value
	ADCSRA |= (1<<ADSC); //Start an ADC conversion upon interrupt
	
} //end ISR1

//pause system interrupt
ISR(INT5_vect){
		
	//delay(20); //debounce
	//while((PINE &0x20) == 0x20);
	//delay(20); //debounce		

	PORTB = DC_HAULT;
	pause_flag = 1;
	pause_press_count++;
	
}

//ramp down system interrupt
ISR(INT6_vect){
	
	//delay(20); //debounce
	//while((PINE &0x40) == 0x40);
	//delay(20); //debounce
	
	blink_LEDs();
	
	OCR2A = 255;
	//start timer 2
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20); //set the prescaler to 1024
	
}

//system ramp down interrupt
ISR(TIMER2_COMPA_vect){
	
	ramp_down_timer_interrupt_count++;
	
	if(ramp_down_timer_interrupt_count >= 35 && total_conveyor_count == 0) ramp_down_system_flag = 1;
}

//stepper timer interrupt
ISR(TIMER1_COMPA_vect){ 

	unsigned int newStepDelay = 0; //stores the new delay
	static int lastAccelDelay = 0; //stores previously used delay
	static unsigned int rest = 0; //stores the calculation remainder for accuracy

	OCR1A = stepper_speed_data.step_delay;

	switch(stepper_speed_data.run_state) {
		
		case STOP:
		
			step_count = 0; //reset step count
			rest = 0; //reset remainder
		
			stepper_movement_status = 0; //reset indicate movement completion
			current_position = required_position; //update current_position
		
			TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); //stop timer/counter 1
		
		break;

		case ACCEL:
	
			send_stepper_pulse(stepper_speed_data.direction); //send a pulse
			step_count++; //update step count
			stepper_speed_data.accel_count++; //update acceleration count
		
			//calculate the next time delay
			newStepDelay = stepper_speed_data.step_delay - (((2 * (long)stepper_speed_data.step_delay) + rest)/(4 * stepper_speed_data.accel_count + 1));
			rest = ((2 * (long)stepper_speed_data.step_delay)+rest)%(4 * stepper_speed_data.accel_count + 1);
		
			//check if we should start deceleration, or if we hit max speed
			if(step_count >= stepper_speed_data.decel_start) {
				stepper_speed_data.accel_count = stepper_speed_data.number_of_decel_steps;
				stepper_speed_data.run_state = DECEL;
				
			} else if(newStepDelay <= stepper_speed_data.min_delay) {
				lastAccelDelay = newStepDelay; //store final accel delay
				newStepDelay = stepper_speed_data.min_delay; //update the delay with max speed
				
				rest = 0; //reset remainder
				stepper_speed_data.run_state = RUN; //update run state
			}
		
		break;

		case RUN:
		
			send_stepper_pulse(stepper_speed_data.direction); //send a pulse
			step_count++; //increase step count
			newStepDelay = stepper_speed_data.min_delay; //update the step delay
		
			//check if we should start deceleration.
			if(step_count >= stepper_speed_data.decel_start) {
				
				stepper_speed_data.accel_count = stepper_speed_data.number_of_decel_steps; //set the decel step counter
												 //which counts up from (-)number
												 //to zero
		
				newStepDelay = lastAccelDelay; //start deceleration with same
												   //delay that accel ended with
				stepper_speed_data.run_state = DECEL; //update run state
			}
		
		break;

		case DECEL:
		
			send_stepper_pulse(stepper_speed_data.direction); //send a pulse
			step_count++; //increase step count
			stepper_speed_data.accel_count++; //increase acceleration count
			
			//calculate the next time delay
			newStepDelay = stepper_speed_data.step_delay - (((2 * (long)stepper_speed_data.step_delay) + rest)/(4 * stepper_speed_data.accel_count + 1));
			rest = ((2 * (long)stepper_speed_data.step_delay)+rest)%(4 * stepper_speed_data.accel_count + 1);

			//check if deceleration is complete
			if(stepper_speed_data.accel_count >= 0){
				stepper_speed_data.run_state = STOP;
			}
		
		break;
		
	} //end switch

	stepper_speed_data.step_delay = newStepDelay; //update the step delay
	
} //end timer1 ISR

//ADC completion interrupt
ISR(ADC_vect){
		
	temp_lower_bits = ADCL;
	temp_upper_bits = ADCH;
	temp_reflective_value = ADC;
	
	if((PIND &0b00000010) == 0b00000010){ //check if the block is in front of the optical sensor
		
		if(temp_reflective_value < reflective_value){ //reassign if the reflective value is lower

			reflective_value = temp_reflective_value;
			upper_bits = temp_upper_bits;
			lower_bits = temp_lower_bits;

		} //end reassignment if

		//Uncomment these for reflective sensor calibration
		//PORTC = lower_bits;
		//PORTA = upper_bits;

		ADCSRA |= (1<<ADSC); //start the next conversion

	} else if(reflective_value == 1023){
		
	} else { //comment classify_block out when calibrating
		classify_block();
	}
} //end ADC completion service routine

//catches unspecified interrupts
ISR(BADISR_vect){

	blink_LEDs();

}
