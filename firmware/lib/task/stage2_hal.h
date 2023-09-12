/*
  stage2_hal.h -- configuration specifically for the Stage2 HAL of the high-oxygen experiment

  Copyright 2023, Robert L. Read

  This program includes free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  See the GNU Affero General Public License for more details.
  You should have received a copy of the GNU Affero General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef STAGE2_HAL_H
#define STAGE2_HAL_H

#include <machine.h>

//#define DISPLAY_ENABLED

//#define RF_FAN 2
//#define _HEATER 3
//#define _STACK DAC0
// This should change to PIN 5 when
// we get the planned control board.
//#define MAX31855_DATA_PIN 5
//#define MAX31850_DATA_PIN 5
// #define RF_FAN_TACH 5
//#define RF_MOSTPLUS_FLOW_PIN A0
//#define RF_MOSTPLUS_FLOW_LOW_CUTOFF_VOLTAGE 1.75
//(MAXCLK, MAXCS, MAXDO);
#define BEEPER_PIN 37 // buzzer pin
//#define U8_MISO 50
//#define U8_MOSI 51
//#define U8_SCK  52
//#define SDSS   53 //sd card ss select
//#define CD     49  //sd card card detect
/*
A0	FAN1_FG			Input		Blower 1 Tachometer	
D4	TEMP1			BIDirect	Dallas One-Wire connection to Thermocouple Breakouts	Daisy chain connection to temprature probes. Address of ???
D9	nFAN1_PWM		Output		Blower PWM	This output will be inverted
D18	TX1	Output		Digital 	Power Supply TF800 Pin 23	This is Serial1 TX for (power supply)[https://assets.alliedelec.com/v1560852133/Datasheets/1d230174086e96b6e4801d1c963649f3.pdf]
D19	RX1	Input		Digital 	Power Supply TF800 Pin 24	This is Serial1 RX for (power supply) [https://assets.alliedelec.com/v1560852133/Datasheets/1d230174086e96b6e4801d1c963649f3.pdf]
D22	BLOWER_ENABLE	Output		Blower Enable	
D32	GPAD_nCS		Output		External SPI inverted select (for the GPAD)	
D44	LPBK0			Output		Varying loopback signal	
D45	LPBK1			Input		Read of digital loopback signal	
D51	HEAT1			Output		Positive SSR signal
D50 Heat2			Output		Positive SSR signal for heaterPIDTask
D49 HEAT3			Output		Positive SSR signal for heater PID 
*/
#define U8_DC 31  //LCD A0  
#define U8_CS 48 //D0LCD_CS
#define U8_RST 38 //LCD_RESET_PIN  
#define U8_Width 128
#define U8_Height 64

//#rst and kill are both pull down can be used to reset mcu. 
//GPIO Defines
#define encA               40
#define encB               24
#define encButton          39 
#define LED_BUILTIN_RED    37 //active high
#define LED_BUILTIN_GREEN  23 //active high
#define LED_BUILTIN_BLUE   53 //active high

// in a perfect world we might use device address
//#define POST_STACK_0_IDX 0
//#define POST_HEATER_0_IDX 1

class Stage2HAL : public MachineHAL {
public:
  bool init() override;
  Stage2Heater s2heaterToControl = Int1;
  const int NUM_HEATERS = 3;
  const int HEATER_PINS[3] = {51,50,49}; // Ext1, Ext2, Int1
  const int NUM_THERMOCOUPLES = 3;
  const int THERMOCOUPLE_PINS[3] = {??,??,??};  // Ext1, Ext2, Int1
  float getTemperatureReading(Stage2Heater s2h,MachineConfig *mc);
};

#endif
