/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 */
public class Robot extends TimedRobot {
  private static final int leadDeviceID = 1;
  private static final int followDeviceID = 2;
  private static final int kJoystickPort = 0;

  private SparkMax leadMotor;
  private SparkMax followMotor;
  private SparkMaxConfig followMotorConfig;
  private SparkMaxConfig emptyConfig;
  private Joystick joystick;

  @Override
  public void robotInit() {
    /*
     * Create two SPARK MAX object with the desired CAN-IDs and types of motors connected
     * MotorType can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    leadMotor = new SparkMax(leadDeviceID, MotorType.kBrushless);
    followMotor = new SparkMax(followDeviceID, MotorType.kBrushless);
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
    emptyConfig = new SparkMaxConfig();
    followMotorConfig = new SparkMaxConfig();

    /*
     * For the follower motor config, we setup follower mode by calling
     * the follow() method from the SparkMaxConfig object.
     * 
     * The following are passed in as arguments:
     *    1. CAN-ID of the leader motor
     *    2. Inversion value for the follower motor
     */
    followMotorConfig.follow(leadDeviceID, false);

    /*
     * After making all the changes in the SparkBaseConfig object (followerMotorConfig in this case), 
     * we apply them to the SPARK MAX by calling the configure() method.
     * 
     * The first argument passed is the SparkBaseConfig object containing any parameter changes we 
     * want applied
     * 
     * The second argument passed is the ResetMode which uses: 
     *    - kResetSafeParameters: Restore defaults before applying parameter changes
     *    - kNoResetSafeParameters:  Don't restore defaults before applying parameter changes
     * 
     * The third argument passed is the PersistMode which uses:
     *    - kNoPersistParameters: Parameters will be not persist over power cycles
     *    - kPersistParameters: Parameters will persist over power cycles
     * 
     * In this case we will be restoring defaults, then applying our parameter values that will not 
     * persist over power cycles for the follower motor. For the leader motor, we will only restore 
     * defaults.
     */
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    leadMotor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Create joystick object to serve as an input for controlling the leader motor
     */
    joystick = new Joystick(kJoystickPort);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * The SparkMax object's set() method is used for duty cycle control and determines the 
     * percentage of the input voltage we want applied on the motor as a decimal (-1 to 1).
     * 
     * Here we are directly using the joystick's y-axis value (-1 to 1) as the duty cycle input.
     * 
     * Since the follower motor is following the leader motor, the follower motor will apply the 
     * same duty cycle as the leader.
     */
    leadMotor.set(joystick.getY());
  }
}
