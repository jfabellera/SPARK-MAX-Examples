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
#include "rev/config/SoftLimitConfig.h"
#include "rev/config/SparkMaxConfig.h"


class Robot : public frc::TimedRobot {
  /**
   * Change these parameters to match your setup
   */
  static constexpr int kDeviceID = 1;
  static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;

  // Initialize the SPARK MAX with device ID and motor type
  rev::spark::SparkMax m_motor{ kDeviceID, kMotorType };

  frc::Joystick m_stick{0};

  bool fwdEnabled{ false }, revEnabled{ false };
  double fwdLimit{ 0.0 }, revLimit{ 0.0 };

 public:
  void RobotInit() {
    rev::spark::SparkMaxConfig maxConfig;

    /**
     * Soft Limits restrict the motion of the motor in a particular direction
     * at a particular point. Soft limits can be applied in only one direction,
     * or both directions at the same time. 
     * 
     * If the soft limits are disabled and then re-enabled, they will retain
     * the last limits that they had for that particular direction.
     */
    maxConfig.softLimit
      .ForwardSoftLimitEnabled(true)
      .ReverseSoftLimitEnabled(true)
      .ForwardSoftLimit(15.0)
      .ReverseSoftLimit(0.0);

    /**
     * The ResetMode::kResetSafeParameters constant can be used to reset
     * the configuration parameters in the SPARK MAX to their factory
     * default state. If PersistMode::kNoPersistParameters is passed,
     * these parameters will not persist between power cycles.
     */
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    fwdEnabled = m_motor.configAccessor.softLimit.GetForwardSoftLimitEnabled();
    revEnabled = m_motor.configAccessor.softLimit.GetReverseSoftLimitEnabled();
    fwdLimit = m_motor.configAccessor.softLimit.GetForwardSoftLimit();
    revLimit = m_motor.configAccessor.softLimit.GetReverseSoftLimit();

    frc::SmartDashboard::PutBoolean("Forward Soft Limit Enabled", fwdEnabled);
    frc::SmartDashboard::PutBoolean("Reverse Soft Limit Enabled", revEnabled);

    frc::SmartDashboard::PutNumber("Forward Soft Limit", fwdLimit);
    frc::SmartDashboard::PutNumber("Reverse Soft Limit", revLimit);
  }

  void TeleopPeriodic() {
    m_motor.Set(m_stick.GetY());

    rev::spark::SparkMaxConfig maxConfig;

    if (bool newFwdEnabled = frc::SmartDashboard::GetBoolean("Forward Soft Limit Enabled", true); newFwdEnabled != fwdEnabled)
    {
        fwdEnabled = newFwdEnabled;
        maxConfig.softLimit.ForwardSoftLimitEnabled(fwdEnabled);
    }
    if (bool newRevEnabled = frc::SmartDashboard::GetBoolean("Reverse Soft Limit Enabled", true); newRevEnabled != revEnabled)
    {
        revEnabled = newRevEnabled;
        maxConfig.softLimit.ReverseSoftLimitEnabled(revEnabled);
    }

    if (bool newFwdLimit = frc::SmartDashboard::GetNumber("Forward Soft Limit", 15); newFwdLimit != fwdLimit)
    {
        fwdLimit = newFwdLimit;
        maxConfig.softLimit.ForwardSoftLimit(fwdLimit);
    }
    if (bool newRevLimit = frc::SmartDashboard::GetNumber("Reverse Soft Limit", 0); newRevLimit != revLimit)
    {
        revLimit = newRevLimit;
        maxConfig.softLimit.ReverseSoftLimit(revLimit);
    }

    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
