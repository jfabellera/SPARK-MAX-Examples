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
#include "rev/config/MAXMotionConfig.h"
#include "rev/config/SparkMaxConfig.h"

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV MAX Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV MAX Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV MAX Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the MAX motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */

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

    rev::spark::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();

  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default MAX motion coefficients
  double kMaxVel = 2000, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  const double MaxRPM = 5700;

  frc::Joystick m_stick{0};

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
     * MAX Motion coefficients are set on a MAXMotionConfig object contained
     * in the SparkMaxConfig.
     * 
     * - MaxVelocity() will limit the velocity in RPM of the PID controller
     * in MAX Motion mode
     * - MaxAcceleration() will limit the acceleration in RPM^2 of the PID
     * controller in MAX Motion mode
     * - AllowedClosedLoopError() will set the max allowed error for the
     * PID controller in MAX Motion mode
     */
    maxConfig.closedLoop.maxMotion
      .MaxVelocity(kMaxVel)
      .MaxAcceleration(kMaxAcc)
      .AllowedClosedLoopError(kAllErr);

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

    // display MAX Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    frc::SmartDashboard::PutNumber("Set Position", 0);
    frc::SmartDashboard::PutNumber("Set Velocity", 0);

    // button to toggle between velocity and MAX motion modes
    frc::SmartDashboard::PutBoolean("Mode", true);
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
    double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

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

    if ((maxV != kMaxVel)) { pidConfig.maxMotion.MaxVelocity(maxV); kMaxVel = maxV; }
    if ((maxA != kMaxAcc)) { pidConfig.maxMotion.MaxAcceleration(maxA); kMaxAcc = maxA; }
    if ((allE != kAllErr)) { pidConfig.maxMotion.AllowedClosedLoopError(allE); allE = kAllErr; }

    rev::spark::SparkMaxConfig maxConfig;
    maxConfig.Apply(pidConfig);
    m_motor.Configure(maxConfig,
        rev::spark::SparkMax::ResetMode::kNoResetSafeParameters,
        rev::spark::SparkMax::PersistMode::kNoPersistParameters);

    double setPoint, processVariable;
    bool mode = frc::SmartDashboard::GetBoolean("Mode", false);
    if(mode) {
      setPoint = frc::SmartDashboard::GetNumber("Set Velocity", 0);
      m_pidController.SetReference(setPoint, rev::spark::SparkMax::ControlType::kVelocity);
      processVariable = m_encoder.GetVelocity();
    } else {
      setPoint = frc::SmartDashboard::GetNumber("Set Position", 0);
      /**
       * As with other PID modes, MAX Motion is set by calling the
       * SetReference method on an existing PID object and setting
       * the control type to kMAXMotionPositionControl or
       * kMAXMotionVelocityControl.
       */
      m_pidController.SetReference(setPoint, rev::spark::SparkMax::ControlType::kMAXMotionPositionControl);
      processVariable = m_encoder.GetPosition();
    }
    
    frc::SmartDashboard::PutNumber("Set Point", setPoint);
    frc::SmartDashboard::PutNumber("Process Variable", processVariable);
    frc::SmartDashboard::PutNumber("Output", m_motor.GetAppliedOutput());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
