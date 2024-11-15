/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private DifferentialDrive myRobot;
  private PS4Controller controller;
  private static final int leftDeviceID = 1; 
  private static final int rightDeviceID = 2;
  private SparkMax leftMotor;
  private SparkMax rightMotor;

  @Override
  public void robotInit() {
    /*
     * Create two SPARK MAX objects by passing in the desired CAN-IDs and types of motors connected
     * Motor types can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    leftMotor = new SparkMax(leftDeviceID, MotorType.kBrushless);
    rightMotor = new SparkMax(rightDeviceID, MotorType.kBrushless);

    /*
     * Create a SparkBaseConfig object that will queue up any changes we want applied to one or more SPARK MAXs.
     * The configuration objects use setter functions that allow for chaining.
     *  
     * Changes made in the config will not get applied to the SPARK MAX until the configure() method is called 
     * from a SPARK MAX object. If a SparkBaseConfig with no changes is passed to the configure() method, the SPARK 
     * MAX's configuration will remain unchanged.
     * 
     * Within the base config, the following can be modified:
     *    - Follower Mode
     *    - Voltage Compensation
     *    - Idle modes
     *    - Motor inversion settings
     *    - Closed/Open Ramp Rates
     *    - Current Limits
     * 
     * The base config also contains sub configs that can be modified such as:
     *    - AbsoluteEncoderConfig
     *    - AlternateEncoderConfig
     *    - Analog Sensor Config
     *    - ClosedLoopConfig
     *    - EncoderConfig
     *    - LimitSwitchConfig
     *    - SoftLimitConfig
     *    - SignalsConfig (Status Frames)
     *  
     * Sub config objects can separately be created, modified and then applied to the base config by calling the apply() method.
     */
    SparkMaxConfig emptyConfig = new SparkMaxConfig();

    /*
     * After making all the changes (none in this example) in the SparkBaseConfig object, 
     * we apply them to the SPARK MAX by calling the configure() method.
     * 
     * The first argument passed is the SparkBaseConfig object containing any parameter changes we 
     * want applied
     * 
     * The second argument passed is the ResetMode which uses: 
     *    - kResetSafeParameters: Restore defaults before applying parameter changes
     *    - kNoResetSafeParameters:  Don't Restore defaults before applying parameter changes
     * 
     * The third argument passed is the PersistMode which uses:
     *    - kNoPersistParameters: Parameters will be not persist over power cycles
     *    - kPersistParameters: Parameters will persist over power cycles
     * 
     * In this case we will only restore defaults without the changes persisting across power cycles
     */
    leftMotor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Create differential drive object to manage the output to the left and right motors
     */
    myRobot = new DifferentialDrive(leftMotor, rightMotor);

    /*
     * Create controller interface for motor inputs
     */
    controller = new PS4Controller(0);

    /*
     * Display default applied output values to setup SmartDashboard tiles
     */
     SmartDashboard.putNumber("Left Motor Applied Output", 0.0);
     SmartDashboard.putNumber("Right Motor Applied Output", 0.0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Update the output to the left and right motors based on the position of the left and right 
     * joystick y-axis values
     */
    myRobot.tankDrive(controller.getLeftY(), controller.getRightY());

    /*
     * Refresh the applied output data on SmartDashboard
     */
    SmartDashboard.putNumber("Left Motor Applied Output", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Motor Applied Output", rightMotor.getAppliedOutput());
  }
}
