/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/smartdashboard.h>
#include "rev/SparkMax.h"
#include "rev/config/AlternateEncoderConfig.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/SparkMaxConfig.h"

class Robot : public frc::TimedRobot {
    /**
     * Change these parameters to match your setup
     */
    static constexpr int kCanID = 1;
    static constexpr auto kMotorType = rev::spark::SparkMax::MotorType::kBrushless;
    static constexpr int kCPR = 8192;

    // initialize SPARK MAX with CAN ID
    rev::spark::SparkMax m_motor{ kCanID, kMotorType };

    rev::spark::SparkMaxAlternateEncoder& m_alternateEncoder = m_motor.GetAlternateEncoder();

    /**
     * A Closed-Loop controller can be constructed in the normal way using the
     * GetClosedLoopController() on the motor you want to control
     */
    rev::spark::SparkClosedLoopController m_pidController = m_motor.GetClosedLoopController();

    // PID coefficients
    double kP = 0.4, kI = 1e-4, kD = 6, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

public:
    void RobotInit() {
        rev::spark::SparkMaxConfig maxConfig;

        /**
         * An alternate encoder always has a type of kQuadrature, so it does
         * not need to be set explicitly. If using a REV Through Bore
         * Encoder, the counts per revolution should be set to 8192.
         */
        maxConfig.alternateEncoder
            /** Configures the data port to use the alternate encoder */
            .SetSparkMaxDataPortConfig()
            .CountsPerRevolution(kCPR);

        /**
         * By default, the PID controller will use the Hall sensor from a NEO for its
         * feedback device. Instead, we can set the feedback device to the alternate
         * encoder object
         */
        maxConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder);

        /**
         * From here on out, code looks exactly like running PID control with the
         * built-in NEO encoder, but feedback will come from the alternate encoder
         */

         // set PID coefficients
        maxConfig.closedLoop
            .Pidf(kP, kI, kD, kFF)
            .IZone(kIz)
            .OutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        frc::SmartDashboard::PutNumber("P Gain", kP);
        frc::SmartDashboard::PutNumber("I Gain", kI);
        frc::SmartDashboard::PutNumber("D Gain", kD);
        frc::SmartDashboard::PutNumber("I Zone", kIz);
        frc::SmartDashboard::PutNumber("Feed Forward", kFF);
        frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
        frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
        frc::SmartDashboard::PutNumber("Set Rotations", 0);

        /**
         * The ResetMode::kResetSafeParameters constant can be used to reset
         * the configuration parameters in the SPARK MAX to their factory
         * default state. If PersistMode::kNoPersistParameters is passed,
         * these parameters will not persist between power cycles.
         */
        m_motor.Configure(maxConfig,
            rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kNoPersistParameters);
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
        double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

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
        m_pidController.SetReference(rotations, rev::spark::SparkMax::ControlType::kPosition);

        frc::SmartDashboard::PutNumber("SetPoint", rotations);
        frc::SmartDashboard::PutNumber("ProcessVariable", m_alternateEncoder.GetPosition());

    }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
