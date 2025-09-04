/*
 * callibrate.h
 *
 *  Created on: May 9, 2025
 *      Author: felix
 */

#ifndef CALLIBRATE_H_
#define CALLIBRATE_H_

#include "fsl_clock.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#define SW3_PORT   PORTC
#define SW3_GPIO   GPIOC
#define SW3_PIN    12U            /* PTC12, active‑low */

void  SW3_Init(void);

uint8_t SW3_IsPressed(void);

void callibrate(int8_t *call, int16_t *x,int16_t *y,int16_t *ox,int16_t *oy);

#endif /* CALLIBRATE_H_ */
