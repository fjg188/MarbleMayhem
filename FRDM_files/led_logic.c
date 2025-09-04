//led_logic.c

#include "led.h"
#include <stdlib.h>

void tilt_logic(int16_t yAngle, int16_t xAngle){ //For sake of program/lights had to flip these two around
	grb32_t led_vals[2]; //LED2 = X tilt, LED1 = Y tilt (based on the way you hold the board: Lights: X Y
	//Based on the accelerometer orientation in board: +x: RIGHT, -x: LEFT, +y: DOWN, -y: UP

	//Clamp angles to +/- 90 degrees as sudden changes can make values > 90 or < -90 (messes with light-tilt-relation)
	if (xAngle > 90) xAngle = 90;
	if (xAngle < -90) xAngle = -90;
	if (yAngle > 90) yAngle = 90;
	if (yAngle < -90) yAngle = -90;

	if (abs(xAngle) < 3) xAngle = 0; //Make a dead-zone to get rid of immediate flickering
	if (abs(yAngle) < 3) yAngle = 0;

	float x_ratio = abs(xAngle) / 90.0f; //To help with the gradient values,
	float y_ratio = abs(yAngle) / 90.0f;

	//X-Tilt Logic (Right (+) = Blue, Left (-) = Yellow, Neutral = White), uses LED2
	if (xAngle > 0) {
		//Right -> Blue
		led_vals[1].red = (uint8_t)(70 * (1 - x_ratio));
		led_vals[1].green = (uint8_t)(70 * (1 - x_ratio));
		led_vals[1].blue = 70;
	}
	else if (xAngle < 0){
		//Left -> Yellow
		led_vals[1].red = 70;
		led_vals[1].green = 70;
		led_vals[1].blue = (uint8_t)(70 * (1 - x_ratio));
	}
	else if (xAngle == 0){
		//Neutral -> White
		led_vals[1].red = 70;
		led_vals[1].green = 70;
		led_vals[1].blue = 70;
	}

	//Y-Tilt Logic (Down (+) = Red, Up (-) = Green, Neutral = White), uses LED1
	if (yAngle > 0) {
	    //+y -> tilt downwards -> Red
	    led_vals[0].red = 70;
	    led_vals[0].green = (uint8_t)(70 * (1 - y_ratio));
	    led_vals[0].blue = (uint8_t)(70 * (1 - y_ratio));
	}
	else if (yAngle < 0){
		//-y -> tilt upwards -> Green
		led_vals[0].red = (uint8_t)(70 * (1 - y_ratio));
		led_vals[0].green = 70;
		led_vals[0].blue = (uint8_t)(70 * (1 - y_ratio));
	}
	else if (yAngle == 0){
		//Neutral -> White
	    led_vals[0].red = 70;
	    led_vals[0].green = 70;
	    led_vals[0].blue = 70;
	}

	// Send both values through serial LED chain
	set_leds(led_vals, 2);
}

//Sequence that turns off LEDs at start of every build
void turn_colors_off(void){
	grb32_t led_vals[2];

	led_vals[0].red = 0;
	led_vals[0].green = 0;
	led_vals[0].blue = 0;

	led_vals[1].red = 0;
	led_vals[1].green = 0;
	led_vals[1].blue = 0;

	set_leds(led_vals, 2);
}
