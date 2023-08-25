/*
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

#include <machine.h>
// there is not yet anything for this to do
#include <core.h>
#include <Wire.h>


void outputReport(MachineStatusReport *msr) {
        OxCore::DebugLn<const char *>("");
        OxCore::Debug<const char *>("Machine State: ");
        OxCore::DebugLn<const char *>(MachineConfig::MachineStateNames[msr->ms]);
        OxCore::Debug<const char *>("Target      C: ");
        OxCore::DebugLn<float>(msr->target_temp_C);
        OxCore::Debug<const char *>("Post Heater C: ");
        OxCore::DebugLn<float>(msr->post_heater_C);
        OxCore::Debug<const char *>("Post Getter C: ");
        OxCore::DebugLn<float>(msr->post_getter_C);
        OxCore::Debug<const char *>("Post Stack  C: ");
        OxCore::DebugLn<float>(msr->post_stack_C);
        OxCore::Debug<const char *>("Stack volts V: ");
        OxCore::DebugLn<float>(msr->stack_voltage);
        OxCore::Debug<const char *>("Stack amps  A: ");
        OxCore::DebugLn<float>(msr->stack_amps);
        OxCore::Debug<const char *>("Stack ohms  O: ");
        if (msr->stack_ohms < 0.0) {
          OxCore::DebugLn<const char*>(" N/A");
        } else {
          OxCore::DebugLn<float>(msr->stack_ohms);
        }
        OxCore::Debug<const char *>("Heater DC    : ");
        Serial.println(msr->heater_duty_cycle,4);
        //        OxCore::DebugLn<float>(msr->heater_duty_cycle);
        OxCore::Debug<const char *>("Fan PWM      : ");
        OxCore::DebugLn<float>(msr->fan_pwm);
        OxCore::Debug<const char *>("Fan RPM      : ");
        OxCore::DebugLn<float>(msr->fan_rpm);
}

void createJSONReport(MachineStatusReport* msr, char *buffer) {
  sprintf(buffer+strlen(buffer), "\"MachineState\": \"%s\",\n",MachineConfig::MachineStateNames[msr->ms]);
  sprintf(buffer+strlen(buffer), "\"TargetC\": \"%.2f\",\n",msr->target_temp_C);
  sprintf(buffer+strlen(buffer), "\"HeaterC\": \"%.2f\",\n",msr->post_heater_C);
  sprintf(buffer+strlen(buffer), "\"StackC\": \"%.2f\",\n",msr->post_stack_C);
  sprintf(buffer+strlen(buffer), "\"GetterC\": \"%.2f\",\n",msr->post_getter_C);
  sprintf(buffer+strlen(buffer), "\"HeaterV\": \"%.2f\",\n",msr->heater_voltage);
  sprintf(buffer+strlen(buffer), "\"StackV\": \"%.2f\",\n",msr->stack_voltage);
  sprintf(buffer+strlen(buffer), "\"StackA\": \"%.2f\",\n",msr->stack_amps);
  if (msr->stack_ohms < 0.0) {
    sprintf(buffer+strlen(buffer), "\"StackOhms\": \"N/A\",\n",msr->stack_amps);
  } else {
    sprintf(buffer+strlen(buffer), "\"StackOhms\": \"%.2f\",\n",msr->stack_ohms);
  }
        // As of the summer of 2023, we are not planning to use a flow sensor
  //  sprintf(buffer+strlen(buffer), "\"FlowMlPerS\": \"%.2f\",\n",msr->flow_ml_per_s);
  sprintf(buffer+strlen(buffer), "\"HeaterDutyCycle\": \"%.2f\",\n",msr->heater_duty_cycle);
  sprintf(buffer+strlen(buffer), "\"FanPWM\": \"%.2f\"\n",msr->fan_pwm);
  sprintf(buffer+strlen(buffer), "\"FanRPM\": \"%.2f\"\n",msr->fan_rpm);
}

bool MachineHAL::init() {
  // we should probably check that we can read this effectively here
  // and return false if not
  if (DEBUG_HAL > 0) {
    Serial.println("HAL: About to run Wire!");
  }
  // We are currently not using I2C
  //   Wire.begin();

  if (DEBUG_HAL > 0) {
      Serial.println("HAL: About to init Fan!");
  }

  _fans[0] = SanyoAceB97("FIRST_FAN",0,RF_FAN,1.0);

  _fans[0]._init();


  _ac_heaters = new GGLabsSSR1*[MachineConfig::NUM_HEATERS];
  for(int i = 0; i < MachineConfig::NUM_HEATERS; i++) {
    _ac_heaters[i] = new GGLabsSSR1();
    _ac_heaters[i]->setHeater(0,LOW);
    _ac_heaters[i]->setHeater(1,LOW);
  }

  if (DEBUG_HAL > 0) {
      Serial.println("HAL:About to return!");
  }
  return true;
}

// updateTheFanSpeed to a percentage of the maximum flow.
// We may have the ability to specify flow absolutely in the future,
// but this is genertic.
void MachineConfig::_updateFanPWM(float unitInterval) {
  for (int i = 0; i < NUM_FANS; i++) {
    hal->_fans[i].update(unitInterval);
  }
  this->report->fan_pwm = unitInterval;
}

// updateTheFanSpeed to a percentage of the maximum flow.
// We may have the ability to specify flow absolutely in the future,
// but this is genertic.
void MachineConfig::_reportFanSpeed() {
  this->report->fan_rpm = hal->_fans[0]._calcRPM(0);
}
