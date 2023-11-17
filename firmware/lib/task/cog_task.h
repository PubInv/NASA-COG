/*
 Copyright (C) 2022 Robert L. Read

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

#ifndef COG_TASK_H
#define COG_TASK_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <core.h>
#include "heater.h"
#include "abstract_ps.h"
#include <machine_core_defs.h>
#include <machine.h>
#include <cog_hal.h>

#include <abstract_temperature.h>

#include <OnePinHeater.h>
#include <MAX31850.h>

#include <heater_pid_task.h>
#include <state_machine_manager.h>
#include <PID_v1.h>
#include <wattage_pid_object.h>
#include <duty_cycle_task.h>

namespace OxApp
{

  // These are the controllable pre-set parameters to the algorithm
// that can be tuned (before the algorithm is running).
   class PreSetParameters  {
   public:
     const float L_w = 180.0; // max stack wattage
     const float M_w = 200.0; // stack wattage at 0 wafer difference
     const float Q_c = 30.0;  // maximum wafer difference
   };
  class OneButtonControl {
  public:
    // Current Stack Wattage
    float W_w = 0;
    // Stack Input Heat (actual wattage - pumping wattage)
    float SIH_w= 0;
    // Computing Pumping Wattage
    float PW_w = 0;
    // Current Fan Speed
    float S_p = 0;
    // Current Total Wattage
    float TW_w = 0;
    // Current Heater Wattage
    float H_w = 0;
    // Current Temperature
    const float T_c = MachineConfig::NOMINAL_AMBIENT_c;
    // Operating Temperature
    const float OT_c = MachineConfig::OPERATING_TEMPERATURE_C;
    // Target Stack Wattage
    float tW_w = 0;
    // Target Heater Wattage
    float tH_w = 0;
    // Target Heater Temperature
    const float tT_c = MachineConfig::OPERATING_TEMPERATURE_C;
    // Target Fan Speed
    float tS_p = 100.0;
    // Heater ramp rate (degrees C per minute)
    const float Hr_Cdm =0.5;
    // Stack Watts ramp rate (watts per minute)
    const float Wr_Wdm = 1;
    // Fan speed ramp rate (% cent per minute)
    const float Sr_Pdm = 2;
    // Substate in terms of pausing due to problems
    // This is an integer representing "how paused" we are.
    // At 1 or more, we are pausing increasting the target temperature.
    // At 2 or more, we are decreasing the fan speed on a schedule.
    int pause_substate = 0;
    unsigned long current_pause_began = 0;

    const float DT_PAUSE_LIMIT_K = 20.0;
    const float PAUSE_TIME_S = 10*60;
    // These are our ohms in the cable and the leads.
    // I'm not entirely sure what this should be.
    const float CABLE_O = 0.1;
    const float NUM_WAFERS = 30;
  };
  class CogTask : public StateMachineManager
  {
  public:
    int PERIOD_MS = 10000;
    int DEBUG_LEVEL = 0;

    OneButtonControl c;
    PreSetParameters p;
    DutyCycleTask *dutyCycleTask;
    WattagePIDObject *wattagePIDObject;

    // There are really several senosrs, but they are indexed!
    const static int NUM_TEMP_SENSORS = 3;
    const static int NUM_TEMP_INDICES = 2;
    const static int NUM_FANS = 1;



    float getTemperatureReadingA_C();
    float getTemperatureReadingB_C();
    float getTemperatureReadingC_C();
    bool updatePowerMonitor();

    // "OneButton Routines"
    float computeHeaterDutyCycleFromWattage(float heaterWattage_w);
    float computeTotalWattage(float controlTemp);
    float computeTargetStackWattage(float targetTotalWattage, float heaterWatts, float currentTemp, float B, float C, float targetStackWatts);
    float computeFanSpeedTargetFromSchedule(float temp);
    float computeFanSpeedTarget(float currentTargetTemp,float temp, float heaterWatts);
    bool heaterWattsAtFullPowerPred(float watts);
    void oneButtonAlgorithm(float &totalWattage_w,float &stackWattage_w,float &heaterWattage_w,float &fanSpeed_p);
    void runOneButtonAlgorithm();
    float computeNernstVoltage(float T_K);
    float computePumpingWork(float T_k,float V,float R_O, float I_A);

    // TODO: I think we should separate all of this
    // computation from state machine by creating a new
    // class for this.

    // The PID controller for OneButton Routine
    PID *pidControllerWattage;
    double totalWattage_Output_W = 0.0;
    double final_totalWattage_W = 0.0;
    double temperatureSetPoint_C = 25.0;
    double input_temperature_C = 25.0;

    // Other "OneButton" stuff
    const int USE_PAUSING = 1;
    int pause_substate = 0;
    const unsigned long curent_pause_began = 0;

    COG_HAL* getHAL();

    void turnOff() override;
    void printGenericInstructions() override;

    float getFanSpeed(float t);
    float computeAmperage(float t);

    void _updateCOGSpecificComponents();
    void _updatePowerComponentsVoltage(float voltage);
    void _configTemperatureSensors();

    void _updateFanSpeed(float percentage);
    void _updateStackVoltage(float voltage);
    void _updateStackAmperage(float amperage);
    void _updateStackWattage(float wattage);

    MachineState _updatePowerComponentsOperation(IdleOrOperateSubState i_or_o) override;
    MachineState _updatePowerComponentsOff() override;
    MachineState _updatePowerComponentsWarmup() override;
    MachineState _updatePowerComponentsIdle() override;
    MachineState _updatePowerComponentsCooldown() override;
    MachineState _updatePowerComponentsCritialFault() override;
    MachineState _updatePowerComponentsEmergencyShutdown() override;
    MachineState _updatePowerComponentsOffUserAck() override;

  private:
    bool _run() override;
    bool _init() override;

  };


}

#endif
