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

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Robot extends TimedRobot {
  private Joystick joystick;
  private static final int deviceID = 1;
  private boolean forwardLimitEnabled;
  private boolean reverseLimitEnabled;
  private double forwardLimitValue;
  private double reverseLimitValue;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder;

  @Override
  public void robotInit() {
    /*
     * Setup starting soft limit values
     */
    forwardLimitEnabled = true;
    reverseLimitEnabled = true;
    forwardLimitValue = 15.0;
    reverseLimitValue = -15.0;

    /*
     * Create a SPARK MAX object with the desired CAN-ID and type of motor connected
     * MotorType can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    motor = new SparkMax(deviceID, MotorType.kBrushless);

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
    motorConfig = new SparkMaxConfig();

    /*
     *  Here we access the softLimit sub config within the SparkMaxBaseConfig to change the
     *  forward and reverse limit parameters by chaining together setter functions.
     *  This is equivalent to:
     *    MotorConfig.softLimit.forwardSoftLimit(forwardLimitValue);
     *    MotorConfig.softLimit.reverseSoftLimit(reverseLimitValue);
     *    MotorConfig.softLimit.forwardSoftLimitEnabled(forwardLimitEnabled);
     *    MotorConfig.softLimit.reverseSoftLimitEnabled(revereseLimitEnabled);
     */
    motorConfig.softLimit
      .forwardSoftLimit(forwardLimitValue)
      .reverseSoftLimit(reverseLimitValue)
      .forwardSoftLimitEnabled(forwardLimitEnabled)
      .reverseSoftLimitEnabled(reverseLimitEnabled);
      
    /*
     * Set the idle mode in the SparkBaseConfig to brake mode
     * There are two idle modes to choose from:
     *    - IdleMode.kBrake : Will short the phases of the motor to stop the rotor immediately and 
     *                        resist change
     *    - IdleMode.kCoast : Will leave phases of the motor phases floating allowing the motor to 
     *                        spin freely until all the power dissapates
     */
    motorConfig.idleMode(IdleMode.kBrake);
    
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
     * persist over power cycles.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Create encoder object by calling the getEncoder() method. Methods like getPosition() and
     * getVelocity() to get position and velocity values from the SPARK MAX Primary Encoder
     */
    encoder = motor.getEncoder();

    /*
     * Display current soft limit parameter values on SmartDashboard
     */
    SmartDashboard.putBoolean("Forward Soft Limit Enabled",
                              motor.configAccessor.softLimit.getForwardSoftLimitEnabled());
    SmartDashboard.putBoolean("Reverse Soft Limit Enabled",
                              motor.configAccessor.softLimit.getReverseSoftLimitEnabled());                          
    SmartDashboard.putNumber("Forward Soft Limit",
                              motor.configAccessor.softLimit.getForwardSoftLimit());
    SmartDashboard.putNumber("Reverse Soft Limit",
                              motor.configAccessor.softLimit.getReverseSoftLimit());
    SmartDashboard.putNumber("Current Position", motor.getEncoder().getPosition());

    /*
     *  Create Joystick object to use the y-axis value as the input to the motor 
     */
    joystick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {

    /*
     * Read soft limit values from SmartDashboard for any changes
     */
    boolean isFwdLimEnabled = SmartDashboard.getBoolean("Forward Soft Limit Enabled", forwardLimitEnabled);
    boolean isRevLimEnabled = SmartDashboard.getBoolean("Reverse Soft Limit Enabled", reverseLimitEnabled);
    double fwdLimit = SmartDashboard.getNumber("Forward Soft Limit", forwardLimitValue);
    double revLimit = SmartDashboard.getNumber("Reverse Soft Limit", reverseLimitValue);

    /*
     *  Update values tracking the current soft limit value if there are any changes
     */
    if(isFwdLimEnabled != forwardLimitEnabled) { 
      motorConfig.softLimit.forwardSoftLimitEnabled(isFwdLimEnabled);
      forwardLimitEnabled = isFwdLimEnabled;  
    }
    if(isRevLimEnabled != reverseLimitEnabled) {
      motorConfig.softLimit.reverseSoftLimitEnabled(isRevLimEnabled);
      reverseLimitEnabled = isRevLimEnabled;
    }
    if(fwdLimit != forwardLimitValue) {
      motorConfig.softLimit.forwardSoftLimit(fwdLimit);
      forwardLimitValue = fwdLimit;
    }
    if(revLimit != reverseLimitValue) {
      motorConfig.softLimit.reverseSoftLimit(revLimit);
      reverseLimitValue = revLimit;
    }

    /*
     * Only apply changes to the SPARK MAX  
     */
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Display the current position of the motor using the primary encoder's getPosition() method
     */
    SmartDashboard.putNumber("Current Position", encoder.getPosition());

    /*
     * The SparkMax object set() method is used for duty cycle control and determines the percentage of the input voltage 
     * we want applied on the motor as a decimal (-1 to 1)
     * 
     * Here we are directly using the Joysticks y-axis value (-1 to 1) as the duty cycle input
     */
    motor.set(joystick.getY());
  }
}
