/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "rev/SparkMax.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/SparkMaxConfig.h"

class Robot : public frc::TimedRobot {

  /**
   * Change these parameters to match your setup
   */
  static constexpr int kDeviceID = 1;
  static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;

  // Initialize the SPARK MAX with device ID and motor type
  rev::spark::SparkMax m_motor{ kDeviceID, kMotorType };

  /**
   * In order to use PID functionality for a controller, a ClosedLoopController object
   * is constructed by calling the GetClosedLoopController() method on an existing
   * SparkMax object.
   */
  rev::spark::SparkClosedLoopController m_pidController = m_motor.GetClosedLoopController();

  // Encoder object created to display position values
  rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();

  frc::Joystick m_stick{0};

  // default PID coefficients
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  // motor max RPM
  const double MaxRPM = 5700;

 public:
  void RobotInit() {
    rev::spark::SparkMaxConfig maxConfig;

    maxConfig.closedLoop
      .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);

    // set PID coefficients
    maxConfig.closedLoop
      .Pidf(kP, kI, kD, kFF)
      .IZone(kIz)
      .OutputRange(kMinOutput, kMaxOutput);

    /**
     * The ResetMode::kResetSafeParameters constant can be used to reset
     * the configuration parameters in the SPARK MAX to their factory
     * default state. If PersistMode::kNoPersistParameters is passed,
     * these parameters will not persist between power cycles.
     */
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }
  void TeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    rev::spark::ClosedLoopConfig pidConfig;
    if ((p != kP)) { pidConfig.P(p); kP = p; }
    if ((i != kI)) { pidConfig.I(i); kI = i; }
    if ((d != kD)) { pidConfig.D(d); kD = d; }
    if ((iz != kIz)) { pidConfig.IZone(iz); kIz = iz; }
    if ((ff != kFF)) { pidConfig.VelocityFF(ff); kFF = ff; }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      pidConfig.OutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }

    rev::spark::SparkMaxConfig maxConfig;
    maxConfig.Apply(pidConfig);
    m_motor.Configure(maxConfig,
      rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
      rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    // read setpoint from joystick and scale by max rpm
    double SetPoint = 0.0;// = MaxRPM*m_stick.GetY();

    if (m_stick.GetRawButton(1)) {
      SetPoint = 100;
    } else if (m_stick.GetRawButton(2)) {
      SetPoint = 500;
    } else if (m_stick.GetRawButton(3)) {
      SetPoint = 750;
    } else if (m_stick.GetRawButton(4)) {
      SetPoint = 2500;
    } else {
      SetPoint = 0;
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  rev::spark::SparkMax::ControlType::kDutyCycle
     *  rev::spark::SparkMax::ControlType::kPosition
     *  rev::spark::SparkMax::ControlType::kVelocity
     *  rev::spark::SparkMax::ControlType::kVoltage
     */
    
    m_pidController.SetReference(SetPoint, rev::spark::SparkMax::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetVelocity());
    
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
