/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This sample program shows how to read basic bus measurements from the SparkMax
 */
public class Robot extends TimedRobot {
  /**
   * Parameters for the SparkMax are defined below. Be sure to change the deviceID
   * and motor type to match your setup.
   */
  private static final int deviceID = 1;
  private static final MotorType motorType = MotorType.kBrushless;
  private static final int kJoystickPort = 0;
  private SparkMax motor;
  private SparkMaxConfig emptyConfig;
  private Joystick joystick;

  @Override
  public void robotInit() {
    /*
     * Create a SPARK MAX object with the desired CAN-ID and type of motor connected
     * MotorType can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    motor = new SparkMax(deviceID, motorType);

    /*
     * Create a SparkBaseConfig object that will queue up any changes we want applied to one or more SPARK MAXs.
     * The configuration objects use setter functions that allow for chaining to keep things neat as shown below.
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
     * In this case we will only be restoring defaults without having them persisting over power cycles.
     */
    motor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Create Joystick as an input for controlling the motor
     */
    joystick = new Joystick(kJoystickPort);
  }

  @Override
  public void robotPeriodic() {
    /* 
     * There are several useful bus measurements you can get from the SPARK MAX by calling the 
     * SparkMax object's getter functions. This includes bus voltage (V), output current (A),
     * Applied Output (duty cycle), and motor temperature (C).
     */
    double busVoltage = motor.getBusVoltage();
    double current = motor.getOutputCurrent();
    double appliedOut = motor.getAppliedOutput();
    double temperature = motor.getMotorTemperature();

    /*
     * Display values read from the SPARK MAX on SmartDashboard
     */
    SmartDashboard.putNumber("Bus Voltage", busVoltage);
    SmartDashboard.putNumber("Current", current);
    SmartDashboard.putNumber("Applied Output", appliedOut);
    SmartDashboard.putNumber("Motor Temperature", temperature);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * The SparkMax object set() method is used for duty cycle control and determines the percentage of the input voltage 
     * we want applied on the motor as a decimal (-1 to 1)
     * 
     * Here we are directly using the Joysticks y-axis value (-1 to 1) as the duty cycle input
     */
    motor.set(joystick.getY());
  }
}
