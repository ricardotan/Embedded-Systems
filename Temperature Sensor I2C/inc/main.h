#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_it.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_gpio.h"
#include "stm32f429i_discovery.h"
#include "stdio.h"
#include "stm32f429i_discovery_lcd.h"
#include "util.h"

//exported types
enum mode{ RUNNING, CHANGE_SP	}; 
//RUNNING :the mode where if the current temp is greater than the temperature set point, the fan will turn on to bring it down below it
//CHANGE_SP:the mode to change the set point temperature

#endif /* __MAIN_H */
