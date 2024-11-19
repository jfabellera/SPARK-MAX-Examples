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
#include "rev/config/LimitSwitchConfig.h"

class Robot : public frc::TimedRobot {
    /**
     * Change these parameters to match your setup
     */
    static constexpr int kDeviceID = 1;
    static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;

    // Initialize the SPARK MAX with device ID and motor type
    rev::spark::SparkMax m_motor{ kDeviceID, kMotorType };

  /**
   * A SparkLimitSwitch object is constructed using the GetForwardLimitSwitch() or
   * GetReverseLimitSwitch() method on an existing SparkMax object, depending
   * on which direction you would like to limit.
   */
  rev::spark::SparkLimitSwitch m_forwardLimit = m_motor.GetForwardLimitSwitch();
  rev::spark::SparkLimitSwitch m_reverseLimit = m_motor.GetReverseLimitSwitch();

  frc::Joystick m_stick{0};

 public:
  void RobotInit() {
    rev::spark::SparkMaxConfig maxConfig;

    /**
     * Limit switches can be configured to one of two polarities :
     *   rev::spark::SparkLimitSwitch::Type::kNormallyOpen
     *   rev::spark::SparkLimitSwitch::Type::kNormallyClosed
     */
    maxConfig.limitSwitch
      .ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyClosed)
      .ReverseLimitSwitchType(rev::spark::LimitSwitchConfig::Type::kNormallyClosed);

    /**
     * Limit switches are enabled by default when the are intialized. They can be disabled
     * by calling enableLimitSwitch(false) on a SparkLimitSwitch object
     * 
     * Limit switches can be reenabled by calling enableLimitSwitch(true)
     * 
     * The isLimitSwitchEnabled() method can be used to check if the limit switch is enabled
     */
    maxConfig.limitSwitch
      .ForwardLimitSwitchEnabled(false)
      .ReverseLimitSwitchEnabled(false);

    /**
     * The ResetMode::kResetSafeParameters constant can be used to reset
     * the configuration parameters in the SPARK MAX to their factory
     * default state. If PersistMode::kNoPersistParameters is passed,
     * these parameters will not persist between power cycles.
     */
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    frc::SmartDashboard::PutBoolean("Forward Limit Enabled", m_motor.configAccessor.limitSwitch.GetForwardLimitSwitchEnabled());
    frc::SmartDashboard::PutBoolean("Reverse Limit Enabled", m_motor.configAccessor.limitSwitch.GetReverseLimitSwitchEnabled());
  }
  void TeleopPeriodic() {
    m_motor.Set(m_stick.GetY());

    /**
     * The Get() method can be used on a SparkLimitSwitch object to read the state of the switch.
     * 
     * In this example, the polarity of the switches are set to normally closed. In this case,
     * Get() will return true if the switch is pressed. It will also return true if you do not 
     * have a switch connected. Get() will return false when the switch is released.
     */
    frc::SmartDashboard::PutBoolean("Forward Limit Switch", m_forwardLimit.Get());
    frc::SmartDashboard::PutBoolean("Reverse Limit Switch", m_forwardLimit.Get());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
