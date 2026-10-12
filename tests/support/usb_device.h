#pragma once

#include <stdint.h>

#define USBD_STATE_CONFIGURED 3U

struct USBD_HandleTypeDef
{
    uint8_t dev_state;
};

extern "C" void MX_USB_DEVICE_Init(void);
