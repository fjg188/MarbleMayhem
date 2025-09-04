/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//This is mainly the Bubble Demo from the Imported SDK demos, but
//it has been modified to deal with negative tilt values

#include "fsl_debug_console.h"
#include "board.h"
#include "math.h"
#include "fsl_mma.h"
#include "fsl_tpm.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "led.h"
#include "callibrate.h"


#include <stdlib.h> //For helping with the positive/negative signs

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_TIMER_BASEADDR TPM0
#define BOARD_FIRST_TIMER_CHANNEL 5U
#define BOARD_SECOND_TIMER_CHANNEL 2U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U
/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85  //Originally had U for unsigned, but we need signed angles
#define ANGLE_LOWER_BOUND 5

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

int8_t callibrated = 0; // tells wether the sw3 button has been pressed or not for callibration

int16_t xoff = 0;
int16_t yoff = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
/* Initialize timer module */
static void Timer_Init(void)
{

    /* convert to match type of data */
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
    tpmParam[0].level = kTPM_LowTrue;
    tpmParam[0].dutyCyclePercent = 0U;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
    tpmParam[1].level = kTPM_LowTrue;
    tpmParam[1].dutyCyclePercent = 0U;

    /* Initialize TPM module */
    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

    CLOCK_SetTpmClock(1U);

    TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
    /* Updated duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}

int main(void)
{
    mma_handle_t mmaHandle = {0};
    mma_data_t sensorData = {0};
    mma_config_t config = {0}; 
    status_t result; 
    uint8_t sensorRange = 0;
    uint8_t dataScale = 0;
    int16_t xData = 0;
    int16_t yData = 0;
    int16_t xAngle = 0;
    int16_t yAngle = 0;
    int16_t xDuty = 0;
    int16_t yDuty = 0;
    uint8_t i = 0;
    uint8_t array_addr_size = 0;

    /* Board pin, clock, debug console init */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();

    /* I2C initialize */
    BOARD_Accel_I2C_Init();
    /* Configure the I2C function */
    config.I2C_SendFunc = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    /* Initialize sensor devices */
    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
    for (i = 0; i < array_addr_size; i++)
    {
        config.slaveAddress = g_accel_address[i];
        /* Initialize accelerometer sensor */
        result = MMA_Init(&mmaHandle, &config);
        if (result == kStatus_Success)
        {
            break;
        }
    }

    if (result != kStatus_Success)
    {
        PRINTF("\r\nSensor device initialize failed!\r\n");
        return -1;
    }
    /* Get sensor range */
    if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
    {
        return -1;
    }
    if (sensorRange == 0x00)
    {
        dataScale = 2U;
    }
    else if (sensorRange == 0x01)
    {
        dataScale = 4U;
    }
    else if (sensorRange == 0x10)
    {
        dataScale = 8U;
    }
    else
    {
    }
    /* Init timer */
    Timer_Init();
    SW3_Init();


    //SIM->CLKDIV1 = 0;     // ensure 15 MHz clock
    //SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  // Ensure PORTE clock enabled
    //PORTE->PCR[3] = PORT_PCR_DSE_MASK | PORT_PCR_MUX(1); // Increase drive strength
    led_init();     // init PTE3
    turn_colors_off();

    //grb32_t red = { .green = 0, .red = 255, .blue = 0 };



    /* Main loop. Get sensor data and update duty cycle */
    while (1){

    	while(callibrated == 0){

    	    		if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
    	    		        {
    	    		            return -1;
    	    		        }

    	    		xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
    	    		yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;

    	    		/* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
    	    		xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
    	    		yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);

    	            Board_UpdatePwm(xDuty, yDuty);

    	            if(SW3_IsPressed()){
    	        		callibrate(&callibrated, &xAngle, &yAngle, &xoff, &yoff);
    	        		//continue;
    	            }
    	            PRINTF("0,0\r\n");
    	           delay(150);
    	    	}
        /* Get new accelerometer data. */
        if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
        {
            return -1;
        }

        /* Get the X and Y data from the sensor data structure in 14 bit left format data*/
        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;

        /* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
        xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
        yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);

        /* Update duty cycle to turn on LEDs when angles ~ 90 */
        if (abs(xAngle) > ANGLE_UPPER_BOUND)
        {
            xDuty = 100;

        }
        if (abs(yAngle) > ANGLE_UPPER_BOUND)
        {
            yDuty = 100;

        }
        /* Update duty cycle to turn off LEDs when angles ~ 0 */
        if (abs(xAngle) < ANGLE_LOWER_BOUND)
        {
            xDuty = 0;

        }
        if (abs(yAngle) < ANGLE_LOWER_BOUND)
        {
            yDuty = 0;
        }

        Board_UpdatePwm(xDuty, yDuty);

        tilt_logic(xAngle, yAngle);

        /* Print out the angle data as a coordinate point)*/
        PRINTF("%d,%d\r\n", xAngle - xoff, yAngle - yoff);
        delay(50);
    }
}
