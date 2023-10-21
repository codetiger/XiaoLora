#ifndef _GAME_TIGER_LORA_H
#define _GAME_TIGER_LORA_H

#ifdef RP2040
#include "sx126x_driver/src/sx126x.h"
#include "sx126x_driver/src/sx126x_hal.h"
#include "common.h"
#else
#include "sx126x.h"
#include "sx126x_hal.h"
#endif

class Lora
{
private:
	sx126x_hal_t context;
public:

    Lora();
    ~Lora();

	void SendData(char* data, uint8_t length);
    void ProcessIrq();
    void RecieveData();
};

#endif