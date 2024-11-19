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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Robot extends TimedRobot {
  private Joystick joystick;
  private static final int deviceID = 1;
  private SparkMax motor;
  private RelativeEncoder encoder;

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
     * In this case, we will be restoring defaults without the change persisting over power cycles.
     */
    motor.configure(emptyConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    /*
     * Create encoder object by calling the getEncoder() method. Methods like getPosition() and
     * getVelocity() to get position and velocity values from the SPARK MAX Primary Encoder
     */
    encoder = motor.getEncoder();

    /*
     * Setup default values for the SmartDashboard tiles.
     */
    SmartDashboard.putNumber("Encoder Position", 0);
    SmartDashboard.putNumber("Encoder Velocity", 0);

    /*
     * Create joystick object to use as an input to control the motor
     */
    joystick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * The SparkMax object's set() method is used for duty cycle control and determines the percentage of the input voltage 
     * we want applied on the motor as a decimal (-1 to 1)
     * 
     * Here we are directly using the Joysticks y-axis value (-1 to 1) as the duty cycle input
     */
    motor.set(joystick.getY());

    /*
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Encoder Position", encoder.getPosition());

    /*
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Encoder Velocity", encoder.getVelocity());
  }
}
