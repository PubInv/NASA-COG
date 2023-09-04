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

#include <Arduino.h>

#include <MAX31850.h>
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

HeaterPIDTask heaterPIDTask_ext1;
HeaterPIDTask heaterPIDTask_ext2;
HeaterPIDTask heaterPIDTask_int1;

DutyCycleTask dutyCycleTask_ext1;
DutyCycleTask dutyCycleTask_ext2;
DutyCycleTask dutyCycleTask_int1;

Stage2HeaterTask stage2HeaterTask_int1;
Stage2HeaterTask stage2HeaterTask_ext1;
Stage2HeaterTask stage2HeaterTask_ext2;

Stage2SerialReportTask stage2SerialReportTask;

Stage2SerialTask stage2SerialTask;



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

  machineConfig->hal = new MachineHAL();

  machineConfig->ms = Off;
  machineConfig->s2sr->ms_int1 = Off;
  machineConfig->s2sr->ms_ext1 = Off;
  machineConfig->s2sr->ms_ext2 = Off;

  bool initSuccess  = machineConfig->hal->init();
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
  cogProperties.state_and_config = (void *) machineConfig;
  delay(1000);
  Serial.println("About to run test!");

  OxCore::TaskProperties readTempsProperties;
  readTempsProperties.name = "readTemps";
  readTempsProperties.id = 20;
  readTempsProperties.period = readTempsTask.PERIOD_MS;
  readTempsProperties.priority = OxCore::TaskPriority::High;
  readTempsProperties.state_and_config = (void *) machineConfig;
  delay(300);
  core.AddTask(&readTempsTask, &readTempsProperties);
  delay(100);

  OxCore::TaskProperties stage2SerialReportProperties;
  stage2SerialReportProperties.name = "stage2SerialReportTemps";
  stage2SerialReportProperties.id = 21;
  stage2SerialReportProperties.period = stage2SerialReportTask.PERIOD_MS;
  stage2SerialReportProperties.priority = OxCore::TaskPriority::High;
  stage2SerialReportProperties.state_and_config = (void *) machineConfig;


  bool stage2SerialReportAdd = core.AddTask(&stage2SerialReportTask, &stage2SerialReportProperties);
  if (!stage2SerialReportAdd) {
     OxCore::Debug<const char *>("stage2SerialReport Task add failed\n");
     abort();
  }

  OxCore::DebugLn<long>(machineConfig->s2sr->ms_int1);
  delay(100);
  OxCore::TaskProperties dutyCycleProperties_int1;
  dutyCycleProperties_int1.name = "dutyCycle_int1";
  dutyCycleProperties_int1.id = 23;
  dutyCycleProperties_int1.period = dutyCycleTask_int1.PERIOD_MS;
  dutyCycleProperties_int1.priority = OxCore::TaskPriority::Low;
  dutyCycleProperties_int1.state_and_config = (void *) machineConfig;
  core.AddTask(&dutyCycleTask_int1, &dutyCycleProperties_int1);
  dutyCycleTask_int1.whichHeater = Int1;

  OxCore::TaskProperties dutyCycleProperties_ext1;
  dutyCycleProperties_ext1.name = "dutyCycle_ext1";
  dutyCycleProperties_ext1.id = 24;
  dutyCycleProperties_ext1.period = dutyCycleTask_ext1.PERIOD_MS;
  dutyCycleProperties_ext1.priority = OxCore::TaskPriority::Low;
  dutyCycleProperties_ext1.state_and_config = (void *) machineConfig;
  core.AddTask(&dutyCycleTask_ext1, &dutyCycleProperties_ext1);
  dutyCycleTask_ext1.whichHeater = Ext1;

  OxCore::TaskProperties dutyCycleProperties_ext2;
  dutyCycleProperties_ext2.name = "dutyCycle_ext2";
  dutyCycleProperties_ext2.id = 25;
  dutyCycleProperties_ext2.period = dutyCycleTask_ext2.PERIOD_MS;
  dutyCycleProperties_ext2.priority = OxCore::TaskPriority::Low;
  dutyCycleProperties_ext2.state_and_config = (void *) machineConfig;
  core.AddTask(&dutyCycleTask_ext2, &dutyCycleProperties_ext2);
  dutyCycleTask_ext1.whichHeater = Ext2;

  OxCore::TaskProperties HeaterPIDProperties_int1;
  HeaterPIDProperties_int1.name = "HeaterPID_int1";
  HeaterPIDProperties_int1.id = 26;
  HeaterPIDProperties_int1.period = heaterPIDTask_int1.PERIOD_MS;
  HeaterPIDProperties_int1.priority = OxCore::TaskPriority::High;
  HeaterPIDProperties_int1.state_and_config = (void *) machineConfig;
  core.AddTask(&heaterPIDTask_int1, &HeaterPIDProperties_int1);
  heaterPIDTask_int1.whichHeater = Int1;

  OxCore::TaskProperties HeaterPIDProperties_ext1;
  HeaterPIDProperties_ext1.name = "HeaterPID_ext1";
  HeaterPIDProperties_ext1.id = 27;
  HeaterPIDProperties_ext1.period = heaterPIDTask_ext1.PERIOD_MS;
  HeaterPIDProperties_ext1.priority = OxCore::TaskPriority::High;
  HeaterPIDProperties_ext1.state_and_config = (void *) machineConfig;
  core.AddTask(&heaterPIDTask_ext1, &HeaterPIDProperties_ext1);
  heaterPIDTask_int1.whichHeater = Ext1;

  OxCore::TaskProperties HeaterPIDProperties_ext2;
  HeaterPIDProperties_ext2.name = "HeaterPID_ext2";
  HeaterPIDProperties_ext2.id = 28;
  HeaterPIDProperties_ext2.period = heaterPIDTask_ext2.PERIOD_MS;
  HeaterPIDProperties_ext2.priority = OxCore::TaskPriority::High;
  HeaterPIDProperties_ext2.state_and_config = (void *) machineConfig;
  core.AddTask(&heaterPIDTask_ext2, &HeaterPIDProperties_ext2);
  heaterPIDTask_int1.whichHeater = Ext1;

  heaterPIDTask_int1.dutyCycleTask = &dutyCycleTask_int1;
  heaterPIDTask_ext1.dutyCycleTask = &dutyCycleTask_ext1;
  heaterPIDTask_ext2.dutyCycleTask = &dutyCycleTask_ext2;

  OxCore::TaskProperties stage2HeaterProperties_int1;
  stage2HeaterProperties_int1.name = "stage2_int1";
  stage2HeaterProperties_int1.id = 29;
  stage2HeaterProperties_int1.period = heaterPIDTask_ext2.PERIOD_MS;
  stage2HeaterProperties_int1.priority = OxCore::TaskPriority::High;
  stage2HeaterProperties_int1.state_and_config = (void *) machineConfig;
  bool stage2HeaterTaskAdded = core.AddTask(&stage2HeaterTask_int1, &stage2HeaterProperties_int1);
  if (!stage2HeaterTaskAdded) {
    OxCore::Debug<const char *>("stage2 int1 Failed\n");
    abort();
  }
  stage2HeaterTask_int1.whichHeater = Int1;

  OxCore::TaskProperties stage2HeaterProperties_ext1;
  stage2HeaterProperties_ext1.name = "stage2_ext1";
  stage2HeaterProperties_ext1.id = 30;
  stage2HeaterProperties_ext1.period = heaterPIDTask_ext2.PERIOD_MS;
  stage2HeaterProperties_ext1.priority = OxCore::TaskPriority::High;
  stage2HeaterProperties_ext1.state_and_config = (void *) machineConfig;
  bool stage2HeaterTaskAdded_ext1 = core.AddTask(&stage2HeaterTask_ext1, &stage2HeaterProperties_ext1);
  if (!stage2HeaterTaskAdded_ext1) {
    OxCore::Debug<const char *>("stage2 ext1 Failed\n");
    abort();
  }
  stage2HeaterTask_ext1.whichHeater = Ext1;

  OxCore::TaskProperties stage2HeaterProperties_ext2;
  stage2HeaterProperties_ext2.name = "stage2_ext2";
  stage2HeaterProperties_ext2.id = 31;
  stage2HeaterProperties_ext2.period = heaterPIDTask_ext2.PERIOD_MS;
  stage2HeaterProperties_ext2.priority = OxCore::TaskPriority::High;
  stage2HeaterProperties_ext2.state_and_config = (void *) machineConfig;
  bool stage2HeaterTaskAdded_ext2 = core.AddTask(&stage2HeaterTask_ext2, &stage2HeaterProperties_ext2);
  if (!stage2HeaterTaskAdded_ext2) {
    OxCore::Debug<const char *>("stage2 ext2 Failedd\n");
    abort();
  }
  stage2HeaterTask_ext2.whichHeater = Ext2;

  stage2HeaterTask_int1.DEBUG_LEVEL = 1;
  stage2HeaterTask_ext1.DEBUG_LEVEL = 1;
  stage2HeaterTask_ext2.DEBUG_LEVEL = 1;

  stage2HeaterTask_int1.STAGE2_TARGET_TEMP = machineConfig->STAGE2_DEFAULT_TEMP_INT1;
  stage2HeaterTask_ext1.STAGE2_TARGET_TEMP = machineConfig->STAGE2_DEFAULT_TEMP_EXT1;
  stage2HeaterTask_ext2.STAGE2_TARGET_TEMP = machineConfig->STAGE2_DEFAULT_TEMP_EXT2;

  stage2HeaterTask_int1.STAGE2_OPERATING_TEMP = machineConfig->STAGE2_OPERATING_TEMP_INT1;
  stage2HeaterTask_ext1.STAGE2_OPERATING_TEMP = machineConfig->STAGE2_OPERATING_TEMP_EXT1;
  stage2HeaterTask_ext2.STAGE2_OPERATING_TEMP = machineConfig->STAGE2_OPERATING_TEMP_EXT2;

  // Now set all the states...
  machineConfig->s2sr->ms_int1 = Off;
  machineConfig->s2sr->ms_ext1 = Off;
  machineConfig->s2sr->ms_ext2 = Off;
  machineConfig->s2heaterToControl = Ext1;

  Serial.println("S2 Heater To Control:");
  Serial.println(machineConfig->s2heaterToControl);
  OxCore::TaskProperties serialProperties;
  serialProperties.name = "serial";
  serialProperties.id = 32;
  serialProperties.period = 250;
  serialProperties.priority = OxCore::TaskPriority::High;
  serialProperties.state_and_config = (void *) machineConfig;
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
