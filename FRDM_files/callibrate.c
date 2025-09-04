
#include "callibrate.h"

void SW3_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortC);

    const port_pin_config_t cfg = {
        .pullSelect          = kPORT_PullUp,
        .slewRate            = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .driveStrength       = kPORT_LowDriveStrength,
        .mux                 = kPORT_MuxAsGpio
    };
    PORT_SetPinConfig(SW3_PORT, SW3_PIN, &cfg);

    gpio_pin_config_t in_cfg = { kGPIO_DigitalInput, 0 };
    GPIO_PinInit(SW3_GPIO, SW3_PIN, &in_cfg);
}

uint8_t SW3_IsPressed(void)
{
    /* active‑low: pressed → 0 on pin */
    return (uint8_t)(!GPIO_PinRead(SW3_GPIO, SW3_PIN));
}

void callibrate(int8_t *call, int16_t *x,int16_t *y,int16_t *ox,int16_t *oy){

	*call = 1;
	*ox = *x;
	*oy = *y;
}
