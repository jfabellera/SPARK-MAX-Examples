/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include "rev/SparkMax.h"
#include "rev/config/SparkMaxConfig.h"
#include "rev/config/SparkMaxConfigAccessor.h"

class Robot : public frc::TimedRobot {
  /**
   * Change these parameters to match your setup
   */
  static constexpr int kDeviceID = 1;
  static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;

  // Initialize the SPARK MAX with device ID and motor type
  rev::spark::SparkMax m_motor{ kDeviceID, kMotorType };

  frc::Joystick m_stick{0};

 public:
  void RobotInit() {
      rev::spark::SparkMaxConfig maxConfig;

    /**
     * Parameters can be set by calling the appropriate method on the SparkMaxConfig object
     * whose properties you want to change.
     */
    maxConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kCoast);

    /**
     * The ResetMode::kResetSafeParameters constant can be used to reset
     * the configuration parameters in the SPARK MAX to their factory
     * default state. If PersistMode::kNoPersistParameters is passed,
     * these parameters will not persist between power cycles.
     */
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    /**
     * Similarly, parameters can be retrieved by calling the appropriate
     * methods on the Accessor object.
     */
    if(m_motor.configAccessor.GetIdleMode() == rev::spark::SparkMaxConfig::IdleMode::kCoast) {
      frc::SmartDashboard::PutString("Idle Mode", "Coast");
    } else {
      frc::SmartDashboard::PutString("Idle Mode", "Brake");
    }

    // Set ramp rate to 0
    maxConfig.OpenLoopRampRate(0);
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    // read back ramp rate value
    frc::SmartDashboard::PutNumber("Ramp Rate", m_motor.configAccessor.GetOpenLoopRampRate());
  }
  
  void TeleopPeriodic() {
    // Set motor output to joystick value
    m_motor.Set(m_stick.GetY());
    
    // periodically read voltage, temperature, and applied output and publish to SmartDashboard
    frc::SmartDashboard::PutNumber("Voltage", m_motor.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Temperature", m_motor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Output", m_motor.GetAppliedOutput());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
