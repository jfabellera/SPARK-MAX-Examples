/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder alternateEncoder;

  @Override
  public void robotInit() {
    /*
     * Initialize the SPARK MAX and get its alternate encoder and closed loop
     * controller objects for later use.
     */
    motor = new SparkMax(1, MotorType.kBrushless);
    alternateEncoder = motor.getAlternateEncoder();
    closedLoopController = motor.getClosedLoopController();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the alternate encoder. The encoder being used in this example
     * is the REV Through Bore Encoder, which has a resolution of 8192 counts
     * per revolution.
     */
    motorConfig.alternateEncoder.countsPerRevolution(8192);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the alternate encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .p(0.001)
        .i(0)
        .d(0)
        .outputRange(-0.2, 0.2);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize target position value on SmartDashboard
    SmartDashboard.setDefaultNumber("Target Position", 0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Get the target position from SmartDashboard and set it as the setpoint
     * for the closed loop controller.
     */
    double targetPosition = SmartDashboard.getNumber("Target Position", 0);
    closedLoopController.setReference(targetPosition, ControlType.kPosition);

    // Display the actual position of the alternate encoder
    SmartDashboard.putNumber("Actual Position", alternateEncoder.getPosition());
  }
}
