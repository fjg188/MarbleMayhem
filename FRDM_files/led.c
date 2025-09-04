

#include <MKL46Z4.h>
#include "led.h"

//Keep these to avoid the bubble demo from activating while running program
const int RED_LED_PIN        = 16;
const int GREEN_LED_PIN      = 6;
const int BLUE_LED_PIN       = 2;
const int RED_LED_PIN_FRDM   = 29;
const int GREEN_LED_PIN_FRDM = 5;
const int SERIAL_LED_PIN     = 3;


void led_init(void) {

	//Clock Port E and Port D
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTD_MASK;


	// set up red LED0
	PORTE->PCR[RED_LED_PIN] = PORT_PCR_MUX(0b001);
	PTE->PSOR = 1 << RED_LED_PIN;  // turn off LED
	PTE->PDDR |= 1 << RED_LED_PIN; // make it output

	// set up green LED0
	PORTE->PCR[GREEN_LED_PIN] = PORT_PCR_MUX(0b001);
	PTE->PSOR = 1 << GREEN_LED_PIN;  // turn off LED
	PTE->PDDR |= 1 << GREEN_LED_PIN; // make it output

	// set up blue LED0
	PORTE->PCR[BLUE_LED_PIN] = PORT_PCR_MUX(0b001);
	PTE->PSOR = 1 << BLUE_LED_PIN;  // turn off LED
	PTE->PDDR |= 1 << BLUE_LED_PIN; // make it output

	// set up red LED pin on FRDM board
	PORTE->PCR[29] = PORT_PCR_MUX(001);
	PTE->PDDR |= 1<<29;
	PTE->PSOR = 1<<29;

	//set up green LED on FRDM board
	PORTD->PCR[5] =  PORT_PCR_MUX(001);
	PTD->PSOR = 1<<5;
	PTD->PDDR |= 1<<5;

	//set up the pin to do the serial for LEDs 1 and 2
	PORTE->PCR[SERIAL_LED_PIN] = PORT_PCR_MUX(0b001);
	PTE->PCOR = 1 << SERIAL_LED_PIN; // set output low
	PTE->PDDR |= 1 << SERIAL_LED_PIN; // make it output

	led_off();
	delay(1);
}

void delay(uint32_t ms){
	for(int i= ms; i>0; i--){
		/* Each clock cycles is 1/48MHz
		 * So per ms there are: 1*10^-3 * 48^10^6 / 3 ~= 16000
		 * loops through the small 3-clock cycle helper function. */
		 loop_3cycles(16000);
	}
}


void red_on(void) {
	PTE->PCOR |= GPIO_PCOR_PTCO(1 << RED_LED_PIN);
}

void red_off(void) {
	PTE->PSOR |= GPIO_PSOR_PTSO(1 << RED_LED_PIN);
}
void red_toggle(void) {
	PTE->PTOR |= GPIO_PTOR_PTTO(1 << RED_LED_PIN);
}

void blue_on(void) {
	PTE->PCOR |= GPIO_PCOR_PTCO(1 <<BLUE_LED_PIN);
}

void blue_off(void) {
	PTE->PSOR |= GPIO_PSOR_PTSO(1 << BLUE_LED_PIN);
}
void blue_toggle(void) {
	PTE->PTOR |= GPIO_PTOR_PTTO(1 << BLUE_LED_PIN);
}

void green_on(void) {
	PTE->PCOR |= GPIO_PCOR_PTCO(1 <<GREEN_LED_PIN);
}

void green_off(void) {
	PTE->PSOR |= GPIO_PSOR_PTSO(1 << GREEN_LED_PIN);
}
void green_toggle(void) {
	PTE->PTOR |= GPIO_PTOR_PTTO(1 << GREEN_LED_PIN);
}

void led0_off(void) {
	red_off();
	blue_off();
	green_off();
}


/*
 * Manipulate red FRDM LED pin
 */
void red_on_frdm(void){
	PTE->PCOR = 1<<29;
}
void red_off_frdm(void){
	PTE->PSOR = 1<<29;
}
void red_toggle_frdm(void){
	PTE->PTOR = 1<<29;
}

/*
 * Manipulate green FRDM LED pin
 */
void green_on_frdm(void){
	PTD->PCOR = 1<<5;
}
void green_off_frdm(void){
	PTD->PSOR = 1<<5;
}
void green_toggle_frdm(void){
	PTD->PTOR = 1<<5;
}

void set_leds(grb32_t * rgb_vals, uint32_t num_led){
	for (uint32_t i = 0; i < num_led; i++){   //Use unit32_t to ensure fixed size, no two's complement, mempry efficiency
		set_led(rgb_vals[i]);
	}
}

void leds_off(uint32_t num_led){
    grb32_t off_color = {0, 0, 0, 0};
    for (uint32_t i = 0; i < num_led; i++){     //Daisy chain turning the LEDS for LED1 and LED2 off using the for loop and num_led
        set_led(off_color);
    }
}
