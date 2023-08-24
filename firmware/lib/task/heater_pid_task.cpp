/*

  heater_pid_ttask.cpp -- A simple PID controller for the SSR AC heater

  Copyright (C) 2023 Robert Read.

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

#include <heater_pid_task.h>


HeaterPIDTask::HeaterPIDTask() {


    /* Setting the initial parameters is always weird for PID controller..
     */

    // These initial values are from Tom Taylor, scaled by the fact
    // that I am using a plant variable of duty cycle on a [0.0..1.0] scale.
    // double FKp = 0.03;
    // double FKi = 0.003;
    // double FKd = 0.0;

    // This "AJUSTMEMNT" was found by experiment
    double ADJUSTMENT = 15.0;
    double FKp = 0.03/ADJUSTMENT;
    // I want zero overshoot, so I am setting this to zero!
    //    double FKi = 0.003/ADJUSTMENT;
    double FKi = 0.0;
    double FKd = 0.0/ADJUSTMENT;

    // Upon after testing, the parameters above, operating at a 60 second
    // sample time, is entirely too high, and allowed an overshoot of 8 degree C
    // in only one second!
    const int SAMPLE_TIME_MS = PERIOD_MS;
    // dutyCycle is measured betwen 0.0 and 1.0 (and ends
    // up being a PWM duty cycle)
    this->pidControllerHeater =
      new PID(&(this->Input_temperature_C), &(this->dutyCycle_Output),
              &(this->HeaterSetPoint_C), FKp, FKi, FKd, DIRECT);
    this->pidControllerHeater->SetOutputLimits(-1.0, 1.0);
    this->pidControllerHeater->SetSampleTime(PERIOD_MS);
    this->pidControllerHeater->SetMode(AUTOMATIC);
  }

  bool HeaterPIDTask::_init()
  {

    OxCore::Debug<const char *>("HeaterPIDTask init\n");
    return true;
  }

  bool HeaterPIDTask::_run()
  {
    OxCore::Debug<const char *>("HeaterPIDTask run\n");

    double previousInput = this->Input_temperature_C;
    this->Input_temperature_C = getConfig()->report->post_heater_C;

    pidControllerHeater->Compute();

    double s = this->dutyCycle_Output + this->final_dutyCycle;

    s = min(s, 1.0);
    s = max(s, 0.0);
    this->final_dutyCycle = s;

    dutyCycleTask->dutyCycle = s;

    // This resets our duty_cycle computation
    dutyCycleTask->reset_duty_cycle();

    getConfig()->report->heater_duty_cycle = dutyCycleTask->dutyCycle;

    // The heater DutyCycle should go into the config, but
    // does not currently exist there!
    //        localGetConfig()->fanPWM = s;

    if (DEBUG_PID > 0) {
      OxCore::Debug<const char *>("previous input ");
      OxCore::DebugLn<double>(previousInput);
      OxCore::Debug<const char *>("Final dutyCycle_Output ");
      OxCore::DebugLn<double>(this->dutyCycle_Output);
      OxCore::Debug<const char *>("Final dutyCycle ");
      OxCore::DebugLn<double>(this->final_dutyCycle);
    }

    return true;
  }