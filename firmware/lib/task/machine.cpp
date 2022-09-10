#include <machine.h>
// there is not yet anything for this to do
#include <core.h>

void outputReport(MachineStatusReport msr) {
        OxCore::Debug<const char *>("Post Heater C: ");
        OxCore::DebugLn<float>(msr.post_heater_C);
        OxCore::Debug<const char *>("Post Stack  C: ");
        OxCore::DebugLn<float>(msr.post_stack_C);
        OxCore::Debug<const char *>("Heater      V: ");
        OxCore::DebugLn<float>(msr.heater_voltage);
        OxCore::Debug<const char *>("Stack       V: ");
        OxCore::DebugLn<float>(msr.stack_voltage);
        OxCore::Debug<const char *>("Flow (ml / s): ");
        OxCore::DebugLn<float>(msr.flow_ml_per_s);
        OxCore::Debug<const char *>("Fan Speed    : ");
        OxCore::DebugLn<float>(msr.fan_speed);
}

bool MachineHAL::init() {
  // we should probably check that we can read this effectively here
  // and return false if not
  _flowsensor = new SensirionFlow();

  if (_flowsensor->flowSensor->serialNumber == 0xFFFFFFFF) {
    Serial.println("FLOW SENSOR NOT AVIALABLE!");
    Serial.println("THIS IS A CRITICAL ERROR!");
    return false;
  }

  _fans[0] = DeltaFans("FIRST_FAN",0,RF_FAN,1.0);
  _fans[0]._init();



  return true;
}

// updateTheFanSpeed to a percentage of the maximum flow.
// We may have the ability to specify flow absolutely in the future,
// but this is genertic.
void MachineHAL::_updateFanSpeed(float unitInterval) {
  for (int i = 0; i < NUM_FANS; i++) {
    _fans[i].update(unitInterval);
  }
}
