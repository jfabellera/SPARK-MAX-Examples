/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/SparkMax.h"
#include "rev/config/SparkMaxConfig.h"

class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are initialized over CAN by constructing a SparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  kMotorType
   *  rev::spark::SparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  static constexpr int kLeftLeadDeviceID = 1, kLeftFollowDeviceID = 2, kRightLeadDeviceID = 3, kRightFollowDeviceID = 4;
  static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;
  rev::spark::SparkMax m_leftLeadMotor{kLeftLeadDeviceID, kMotorType};
  rev::spark::SparkMax m_rightLeadMotor{kRightLeadDeviceID, kMotorType};
  rev::spark::SparkMax m_leftFollowMotor{kLeftFollowDeviceID, kMotorType};
  rev::spark::SparkMax m_rightFollowMotor{kRightFollowDeviceID, kMotorType};

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow
   * m_leftLeadMotor and m_rightLeadMotor, respectively.
   *
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are
   * sent to them will automatically be copied by the follower motors
   */
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick{0};
  
 public:
  void RobotInit() {
    /**
     * The ResetMode::kResetSafeParameters constant can be used to reset
     * the configuration parameters in the SPARK MAX to their factory
     * default state. If PersistMode::kNoPersistParameters is passed,
     * these parameters will not persist between power cycles.
     */
    rev::spark::SparkMaxConfig leftLeadConfig;

    m_leftLeadMotor.Configure(leftLeadConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    rev::spark::SparkMaxConfig rightLeadConfig;

    m_rightLeadMotor.Configure(rightLeadConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX Config you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    rev::spark::SparkMaxConfig leftFollowConfig;
    leftFollowConfig.Follow(m_leftLeadMotor);

    m_leftFollowMotor.Configure(leftFollowConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    rev::spark::SparkMaxConfig rightFollowConfig;
    rightFollowConfig.Follow(m_rightLeadMotor);

    m_rightFollowMotor.Configure(rightFollowConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);
  }

  void TeleopPeriodic() {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
