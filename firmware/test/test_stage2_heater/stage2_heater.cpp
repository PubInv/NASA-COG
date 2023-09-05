/*
  Stage2_heater -- A program to control 3 heaters

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

#include <unity.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cstdint>
#include <debug.h>
#include <OnePinHeater.h>
using namespace OxCore;


#include <core_defines.h>
#include <core.h>
#include <machine.h>
#include <stage2_config.h>
#include <stage2_hal.h>

#include <Arduino.h>

// #include <MAX31850.h>
#include <duty_cycle_task.h>
#include <heater_pid_task.h>
#include <read_temps_task.h>
#include <stage2_heater_task.h>
#include <stage2SerialReportTask.h>
#include <serial_task.h>

#include <menuIO/U8x8Out.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include <menuIO/clickEncoderIn.h>
using namespace Menu;

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
#define U8_DC 16  //LCD A0  
#define U8_CS 17 //D0LCD_CS
#define U8_RST 23 //LCD_RESET_PIN  
#define U8_Width 128
#define U8_Height 64

//#rst and kill are both pull down can be used to reset mcu. 
//GPIO Defines
#define encA               31
#define encB               33
#define encButton          35 
#define LED_BUILTIN_RED    25 //active high
#define LED_BUILTIN_GREEN  27 //active high
#define LED_BUILTIN_BLUE   29 //active high

U8X8_PCD8544_84X48_4W_HW_SPI u8x8(U8_CS, U8_DC , U8_RST);

const char* constMEM hexDigit MEMMODE="0123456789ABCDEF";
const char* constMEM hexNr[] MEMMODE={"0","x",hexDigit,hexDigit};
char buf1[]="0x11";//<-- menu will edit this text

ClickEncoder clickEncoder(encA,encB,encButton,2);
ClickEncoderStream encStream(clickEncoder,1);
MENU_INPUTS(in,&encStream);
void timerIsr() {clickEncoder.service();}

//define colors
#define BLACK {0,0,0}
#define BLUE {0,0,255}
#define GRAY {128,128,128}
#define WHITE {255,255,255}
#define YELLOW {255,255,0}
#define RED {255,0,0}

const colorDef<rgb> my_colors[6] {
  {{BLACK,BLACK},{BLACK,BLUE,BLUE}},//bgColor
  {{GRAY,GRAY},{WHITE,WHITE,WHITE}},//fgColor
  {{WHITE,BLACK},{YELLOW,YELLOW,RED}},//valColor
  {{WHITE,BLACK},{WHITE,YELLOW,YELLOW}},//unitColor
  {{WHITE,GRAY},{BLACK,BLUE,WHITE}},//cursorColor
  {{WHITE,YELLOW},{BLUE,RED,RED}},//titleColor
};

#define offsetX 0
#define offsetY 0

#define MAX_DEPTH 2

MENU_OUTPUTS(out,MAX_DEPTH
  ,SERIAL_OUT(Serial)
  ,U8X8_OUT(u8x8,{0,0,10,6})
);

NAVROOT(nav,mainMenu,MAX_DEPTH,serial,out);

using namespace OxCore;
static Core core;

const unsigned long REPORT_PERIOD_MS = 5000;
TaskProperties _properties;
unsigned long time_of_last_report = 0;
MachineConfig *machineConfig;

// This is a key parameter, which should perhaps be moved to a specific
// default file to make it clearer!
const float STAGE2_DEFAULT_TEMP = 25.0;
const float STAGE2_OPERATING_TEMP = 750.0;
using namespace std;


using namespace OxApp;
ReadTempsTask readTempsTask;

HeaterPIDTask heaterPIDTask[3];
DutyCycleTask dutyCycleTask[3];
Stage2HeaterTask stage2HeaterTask[3];

Stage2SerialReportTask stage2SerialReportTask;

Stage2SerialTask stage2SerialTask;

MachineConfig *getConfig() {
  return machineConfig;
}

void setup() {

 OxCore::serialBegin(115200UL);
  delay(500);

  if (core.Boot() == false) {
    ErrorHandler::Log(ErrorLevel::Critical, ErrorCode::CoreFailedToBoot);
    // TODO: Output error message
    //return EXIT_FAILURE;
    return;
  }

  encoderIn<encA,encB> encoder;//simple quad encoder driver
  
  
  
  machineConfig = new MachineConfig();

  getConfig()->hal = new Stage2HAL();

  getConfig()->ms = Off;
  getConfig()->s2sr->ms[Int1] = Off;
  getConfig()->s2sr->ms[Ext1] = Off;
  getConfig()->s2sr->ms[Ext2] = Off;

  bool initSuccess  = getConfig()->hal->init();
  Serial.println("about to start!");
  if (!initSuccess) {
    Serial.println("Could not init Hardware Abastraction Layer Properly!");
  }

  //Hardware enable
  pinMode(RF_HEATER, OUTPUT);
  pinMode(encButton,INPUT_PULLUP);
  pinMode(LED_BUILTIN_RED,OUTPUT);
  pinMode(LED_BUILTIN_GREEN,OUTPUT);
  pinMode(LED_BUILTIN_BLUE,OUTPUT);
  
  OxCore::TaskProperties cogProperties;
  cogProperties.name = "cog";
  cogProperties.id = 20;
  cogProperties.state_and_config = (void *) getConfig();
  delay(1000);
  Serial.println("About to run test!");

  OxCore::TaskProperties readTempsProperties;
  readTempsProperties.name = "readTemps";
  readTempsProperties.id = 20;
  readTempsProperties.period = readTempsTask.PERIOD_MS;
  readTempsProperties.priority = OxCore::TaskPriority::High;
  readTempsProperties.state_and_config = (void *) getConfig();
  delay(300);
  core.AddTask(&readTempsTask, &readTempsProperties);
  delay(100);

  OxCore::TaskProperties stage2SerialReportProperties;
  stage2SerialReportProperties.name = "stage2SerialReportTemps";
  stage2SerialReportProperties.id = 21;
  stage2SerialReportProperties.period = stage2SerialReportTask.PERIOD_MS;
  stage2SerialReportProperties.priority = OxCore::TaskPriority::High;
  stage2SerialReportProperties.state_and_config = (void *) getConfig();


  bool stage2SerialReportAdd = core.AddTask(&stage2SerialReportTask, &stage2SerialReportProperties);
  if (!stage2SerialReportAdd) {
     OxCore::Debug<const char *>("stage2SerialReport Task add failed\n");
     abort();
  }

  for(int i = 0; i < 3; i++) {
    OxCore::TaskProperties dutyCycleProperties;
    dutyCycleProperties.name = "dutyCycle";
    dutyCycleProperties.id = 23+i;
    dutyCycleProperties.period = dutyCycleTask[i].PERIOD_MS;
    dutyCycleProperties.priority = OxCore::TaskPriority::Low;
    dutyCycleProperties.state_and_config = (void *) getConfig();
    core.AddTask(&dutyCycleTask[i], &dutyCycleProperties);
    dutyCycleTask[i].whichHeater = (Stage2Heater) i;

    OxCore::TaskProperties HeaterPIDProperties;
    HeaterPIDProperties.name = "HeaterPID";
    HeaterPIDProperties.id = 26+i;
    HeaterPIDProperties.period = heaterPIDTask[i].PERIOD_MS;
    HeaterPIDProperties.priority = OxCore::TaskPriority::High;
    HeaterPIDProperties.state_and_config = (void *) getConfig();
    core.AddTask(&heaterPIDTask[i], &HeaterPIDProperties);
    heaterPIDTask[i].whichHeater = (Stage2Heater) i;

    dutyCycleTask[i].one_pin_heater = getConfig()->hal->_ac_heaters[i];

    OxCore::TaskProperties stage2HeaterProperties ;
    // I really need to do a concatenation here to make the name complete.
    stage2HeaterProperties.name = getConfig()->HeaterNames[i];
    stage2HeaterProperties.id = 29+i;
    stage2HeaterProperties.period = stage2HeaterTask[i].PERIOD_MS;
    stage2HeaterProperties.priority = OxCore::TaskPriority::High;
    stage2HeaterProperties.state_and_config = (void *) getConfig();
    bool stage2HeaterTaskAdded = core.AddTask(&stage2HeaterTask[i], &stage2HeaterProperties);
    if (!stage2HeaterTaskAdded) {
      OxCore::Debug<const char *>("stage add Failed\n");
      abort();
    }
    stage2HeaterTask[i].whichHeater = (Stage2Heater) i;

    stage2HeaterTask[i].STAGE2_TARGET_TEMP = getConfig()->TARGET_TEMP;
    stage2HeaterTask[i].STAGE2_OPERATING_TEMP = getConfig()->STAGE2_OPERATING_TEMP[i];
    getConfig()->s2sr->ms[i] = Off;

    heaterPIDTask[i].dutyCycleTask = &dutyCycleTask[i];
  }

  // Let's put our DEBUG_LEVELS here...
  for(int i = 0; i < 3; i++) {
    stage2HeaterTask[i].DEBUG_LEVEL = 1;
  }


  // This is used to determine which machine will
  // be set by keyboard commands; you switch between them
  // with simple commands
  getConfig()->s2heaterToControl = Int1;

  Serial.println("S2 Heater To Control:");
  Serial.println(getConfig()->s2heaterToControl);
  OxCore::TaskProperties serialProperties;
  serialProperties.name = "serial";
  serialProperties.id = 32;
  serialProperties.period = 250;
  serialProperties.priority = OxCore::TaskPriority::High;
  serialProperties.state_and_config = (void *) getConfig();
  bool serialAdd = core.AddTask(&stage2SerialTask, &serialProperties);
  if (!serialAdd) {
    OxCore::Debug<const char *>("SerialProperties add failed\n");
    abort();
  }

  OxCore::Debug<const char *>("Added tasks\n");
  
  OxCore::Debug<const char *>("Heater PID Controller");
  lcd.begin(20,4);
  nav.idleTask=idle;//point a function to be used when menu is suspended
  mainMenu[1].enabled=disabledStatus;
  nav.showTitle=false;
  lcd.setCursor(0, 0);
  lcd.print("Heater PID Controller");
  lcd.setCursor(0, 1);
  lcd.print("r-site.net");
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
}

void loop() {
  OxCore::Debug<const char *>("Loop starting...\n");
  nav.poll();
  digitalWrite(LED_BUILTIN_RED, ledCtrlRed);
  digitalWrite(LED_BUILTIN_GREEN, ledCtrlGreen);
  digitalWrite(LED_BUILTIN_BLUE, ledCtrlBlue);
  delay(100);
  // Blocking call
  if (core.Run() == false) {
    OxCore::ErrorHandler::Log(OxCore::ErrorLevel::Critical, OxCore::ErrorCode::CoreFailedToRun);
    OxCore::Debug<const char *>("Aborting! TEST OVER\n");
    delay(300);
#ifdef ARDUINO
    // Loop endlessly to stop the program from running
    OxCore::Debug<const char *>("Aborting! TEST OVER\n");
    delay(300);
    abort();
    // while (true) {}
#endif
    return;
  } else {
    OxCore::Debug<const char *>("Run Completely\n");
  }

}

#ifndef ARDUINO
int main(int argc, char **argv)
{
  setup();
  loop();
  abort();
}
#endif
