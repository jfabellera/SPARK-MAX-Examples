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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  /**
   * Change these parameters to match your setup
   */
  private static final int kCanID = 1;
  private static final MotorType kMotorType = MotorType.kBrushless;
  private static final int kCPR = 8192;

  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private RelativeEncoder alternateEncoder;

  @Override
  public void robotInit() {
    /*
     * Define starting PID coefficients
     */
    kP = 0.009;
    kI = 0.0000002;
    kD = 0.00001;
    kIz = 0;
    kFF = 0.00001;
    kMaxOutput = 0.2;
    kMinOutput = -0.2;

    /*
     * Create a SPARK MAX object with the desired CAN-ID and type of motor connected
     * MotorType can be either:
     *    - MotorType.kBrushless
     *    - MotorType.kBrushed
     */
    motor = new SparkMax(kCanID, kMotorType);

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
     * Using the alternateEncoder sub config within the  we can configure the alternate encoder's 
     * counts per revolution by calling the countsPerRevolution() setter function.
     * 
     * Other alternate encoder parameters like position or velocity conversion factors can be changed 
     * through their respective setter functions.
     */
    motorConfig.alternateEncoder.countsPerRevolution(kCPR);

    /*
     * Here we access the closedLoop sub config within the SparkBaseConfig object to change the
     * feedback sensor and PID values we want by chaining together setter functions. This is 
     * equivalent to:
     *    MotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
     *    MotorConfig.closedLoop.p(kP);
     *    MotorConfig.closedLoop.i(kI);
     *    ... 
     *    MotorConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
     */
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

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
     * In this case we will be restoring defaults, then applying our parameter values without having
     * them persisting over power cycles.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  /*
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing SparkMax object. If using a REV Through Bore 
   * Encoder, the counts per revolution should be set to 8192.
   */
    alternateEncoder = motor.getAlternateEncoder();
    
    /**
     * In order to use closed loop functionality for a controller, a SparkClosedLoopController object
     * is constructed by calling the getClosedLoopController() method on an existing
     * SparkMax object
     */
    closedLoopController = motor.getClosedLoopController();

    /*
     *  Display Starting PID coefficients
     */
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Applied Output", 0.0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Read PID coefficients from SmartDashboard for any PID coefficient changes
     */
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    /*
     * If there are any PID value changes, update variables tracking the current value
     */
    if((p != kP)) { motorConfig.closedLoop.p(p); kP = p; }
    if((i != kI)) { motorConfig.closedLoop.i(i); kI = i; }
    if((d != kD)) { motorConfig.closedLoop.d(d); kD = d; }
    if((iz != kIz)) { motorConfig.closedLoop.iZone(iz); kIz = iz; }
    if((ff != kFF)) { motorConfig.closedLoop.velocityFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      motorConfig.closedLoop.outputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /*
     * Apply any changes to the SPARK MAX
     */
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * ClosedLoopController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.spark.SparkBase.ControlType.kDutyCycle
     *  com.revrobotics.spark.SparkBase.ControlType.kPosition
     *  com.revrobotics.spark.SparkBase.ControlType.kVelocity
     *  com.revrobotics.spark.SparkBase.ControlType.kVoltage
     */
    closedLoopController.setReference(rotations, SparkMax.ControlType.kPosition);
    
    /*
     * Display our current setpoint target and processVariable
     */
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", alternateEncoder.getPosition());
  }
}