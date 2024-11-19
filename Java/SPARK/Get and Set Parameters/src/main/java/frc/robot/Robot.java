/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class Robot extends TimedRobot {
  private Joystick stick;
  private static final int deviceID = 1;
  private static final double positionConvFactorDefault = 1.0f;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;

  @Override
  public void robotInit() {
    /*
     * Create a SPARK MAX object with the desired CAN-ID and type of motor connected
     * MotorType can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    
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
    motorConfig = new SparkMaxConfig();

    /*
     * Use SparkBaseConfig to configure the motor idle mode to kCoast and inversion value to false.
     * There are two idle modes to choose from:
     *    - IdleMode.kBrake : Will short the phases of the motor to stop the rotor immediately and 
     *                        resist change
     *    - IdleMode.kCoast : Will leave phases of the motor phases floating allowing the motor to 
     *                        spin freely until all the power dissapates
     */
    motorConfig
      .idleMode(IdleMode.kCoast)
      .inverted(false);

    /*
     * Use the encoder sub config to configure the position conversion factor
     * This could be used to scale or convert the position value to a different metric
     * other than revolutions
     */
    motorConfig.encoder
      .positionConversionFactor(positionConvFactorDefault);

    /*
     * After making all the changes in the SparkBaseConfig object (motorConfig in this case), 
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
     * In this case we will be restoring defaults, then applying our parameter values that will not 
     * persist over power cycles and storing the status of the configuration operation.
     */
    REVLibError status = motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    if(status != REVLibError.kOk) {
      /*
       * On Error, display error messages on Smart Dashboard.
       */
      SmartDashboard.putString("Idle Mode", "Error");
      SmartDashboard.putString("Inverted", "Error");
      SmartDashboard.putString("Position Conversion Factor", "Error");
    } else {
      /*
       * On success, read parameter values from the SPARK MAX to display on the Smart Dashboard 
       * by using the configAccessor and its getter methods. 
       */
      if(motor.configAccessor.getIdleMode() == IdleMode.kCoast) {
        SmartDashboard.putString("Idle Mode", "Coast");
      } else {
        SmartDashboard.putString("Idle Mode", "Brake");
      }
      SmartDashboard.putBoolean("Inverted", motor.configAccessor.getInverted());
      SmartDashboard.putNumber("Position Conversion Factor", motor.configAccessor.encoder.getPositionConversionFactor());
    }

    stick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * The SparkMax object set() method is used for duty cycle control and determines the percentage of the input voltage 
     * we want applied on the motor as a decimal (-1 to 1).
     * 
     * Here we are directly using the Joysticks y-axis value (-1 to 1) as the duty cycle input.
     */
    motor.set(stick.getY());
    
    /*
     * Periodically read voltage, temperature, and applied output to update values on SmartDashboard
     * by calling their respective getter functions from the SparkMax object.
     */
    SmartDashboard.putNumber("Voltage", motor.getBusVoltage());
    SmartDashboard.putNumber("Temperature", motor.getMotorTemperature());
    SmartDashboard.putNumber("Output", motor.getAppliedOutput());
  }
}
