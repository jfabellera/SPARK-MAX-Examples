/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Joystick joystick;
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkLimitSwitch forwardLimit;
  private SparkLimitSwitch reverseLimit;

  private boolean isForwardLimitEnabled;
  private boolean isReverseLimitEnabled;
  private Type forwardLimitNormalState;
  private Type reverseLimitNormalState;

  @Override
  public void robotInit() {
    /*
     * Setup starting values for the limit switch settings
     */
    isForwardLimitEnabled = false;
    isReverseLimitEnabled = false;
    forwardLimitNormalState = Type.kNormallyOpen;
    reverseLimitNormalState = Type.kNormallyOpen;

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
     * A SparkLimitSwitch object is constructed using the getForwardLimitSwitch() or
     * getReverseLimitSwitch() method on an existing SparkMax object. These can be used
     * for getting the current of the forward or reverse limit switch.
     */
    forwardLimit = motor.getForwardLimitSwitch();
    reverseLimit = motor.getReverseLimitSwitch();

    /*
     * Using the limitSwitch sub config, we can adjust the enabled state by calling the 
     * forwardLimitSwitchEnabled() or reverseLimitSwitchEnabled() methods as well as set
     * the switch type by calling the forwardLimitSwitchType() or reverseLimitSwitchType() 
     * methods.
     * 
     * There are two polarities that can be configured for the switch type:
     *    - Type.kNormallyOpen
     *    - Type.KNormallyClosed
     */
    motorConfig.limitSwitch
      .forwardLimitSwitchEnabled(isForwardLimitEnabled)
      .forwardLimitSwitchType(forwardLimitNormalState)
      .reverseLimitSwitchEnabled(isReverseLimitEnabled)
      .reverseLimitSwitchType(reverseLimitNormalState);

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

    joystick = new Joystick(0);

    /*
     * Display starting limit switch parameter values on SmartDashboard
     */
    SmartDashboard.putBoolean("Forward Limit Enabled", isForwardLimitEnabled);
    SmartDashboard.putBoolean("Reverse Limit Enabled", isReverseLimitEnabled);

    /**
     * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the 
     * switch. In this case, isPressed() will return true if the switch is pressed and false when 
     * the switch is released.
     */
    SmartDashboard.putBoolean("Forward Limit Activated", forwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Activated", reverseLimit.isPressed());
  }

  @Override
  public void teleopPeriodic() {
    motor.set(joystick.getY());
    /*
     * Read the current values of the limit switches from Smart Dashboard for any changes
     */
    boolean fwLimEnabled = SmartDashboard.getBoolean("Forward Limit Enabled", false);
    boolean revLimEnabled = SmartDashboard.getBoolean("Reverse Limit Enabled", false); 

    /*
     *  Update values tracking the current limit switch parameter values if there are any changes
     */
    if(isForwardLimitEnabled != fwLimEnabled) {
      isForwardLimitEnabled = fwLimEnabled;
      motorConfig.limitSwitch.forwardLimitSwitchEnabled(isForwardLimitEnabled);
      SmartDashboard.putBoolean("Forward Limit Enabled", isForwardLimitEnabled);
    } 
    if(isReverseLimitEnabled != revLimEnabled) {
      isReverseLimitEnabled = revLimEnabled;
      motorConfig.limitSwitch.reverseLimitSwitchEnabled(isReverseLimitEnabled);
      SmartDashboard.putBoolean("Reverse Limit Enabled", isReverseLimitEnabled);
    }

    /*
     * Only apply the changes to the SPARK MAX
     */
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Update SmartDashboard on whether the limit switch are pressed or not
     */
    SmartDashboard.putBoolean("Forward Limit Activated", forwardLimit.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Activated", reverseLimit.isPressed());
  }
}
