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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV MAXMotion Guide
 * 
 * The SPARK MAX includes a new control mode, REV MAXMotion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV MaxMotion Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the MAXMotion parameters
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
public class Robot extends TimedRobot {
  private static final int deviceID = 1;
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  @Override
  public void robotInit() {
    /*
     * Starting PID coefficients
     */
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    /*
     * Starting MAXMotion Coefficients
     */
    maxVel = 2000; // RPM
    maxAcc = 1500; // RPM^2
    allowedErr = 0.1;

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
     *  Here we access the closedLoop sub config within the SparkBaseConfig to change the
     *  feedback sensor and PID values we want by chaining together setter functions.
     *  This is equivalent to:
     *    MotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
     *    MotorConfig.closedLoop.p(kP);
     *    MotorConfig.closedLoop.i(kI);
     *    ... 
     *    MotorConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
     */
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .iZone(kIz)
      .velocityFF(kFF)
      .outputRange(kMinOutput, kMaxOutput);

    /*
     * The maximum velocity, acceleration, and allowed error threshold within the
     * closed loop controller can be adjusted by calling the following methods in the
     * MaxMotionConfig object within the closedLoop sub config:
     * 
     *    - maxVelocity() will limit the velocity in RPM of the closed loop controller
     *    - maxAccel() will limit the acceleration in RPM^2 of the closed loop controller
     *    - allowedClosedLoopError() will set the max allowed error for the closed loop controller
     */
    motorConfig.closedLoop.maxMotion
      .allowedClosedLoopError(allowedErr)
      .maxAcceleration(maxAcc)
      .maxVelocity(maxVel)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

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
     * Create closed loop controller object that is used for PID functionality
     */
    closedLoopController = motor.getClosedLoopController();

    /*
     * Create encoder object by calling the getEncoder() method. Methods like getPosition() and
     * getVelocity() to get position and velocity values from the SPARK MAX Primary Encoder
     */
    encoder = motor.getEncoder();

    /*
     *  Display Starting PID coefficients and MAXMotion coefficients
     */
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    /*
     * Display a button to toggle between velocity and position MAXMotion modes
     */
    SmartDashboard.putBoolean("Mode", true);
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
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    double setPoint, processVariable;

    /*
     * If there are any PID or MAXMotion parameter changes, update variables tracking the current value
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
    if((maxV != maxVel)) { motorConfig.closedLoop.maxMotion.maxVelocity(maxV); maxVel = maxV; }
    if((maxA != maxAcc)) { motorConfig.closedLoop.maxMotion.maxAcceleration(maxA); maxAcc = maxA; }
    if((allE != allowedErr)) { motorConfig.closedLoop.maxMotion.allowedClosedLoopError(allE); allowedErr = allE; }

    /*
     * Apply any changes to the SPARK MAX
     */
    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    /**
     * As with other PID modes, MAXMotion is set by calling the setReference() method on an 
     * existing closed loop control object and setting the control type to: 
     *    - kMAXMotionVelocityControl
     *    - kMAXMotionPositionControl
     * 
     * We also get the current position or velocity of the motor by calling the getPosition()
     * or getVelocity() encoder methods depending on the mode 
     */
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      closedLoopController.setReference(setPoint, SparkMax.ControlType.kMAXMotionVelocityControl);
      processVariable = encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      closedLoopController.setReference(setPoint, SparkMax.ControlType.kMAXMotionPositionControl);
      processVariable = encoder.getPosition();
    }
    
    /*
     * Display our current setpoint target, processVariable, and motor applied output
     */
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", motor.getAppliedOutput());
  }
}
