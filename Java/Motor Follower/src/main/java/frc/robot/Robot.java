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
  private Joystick joystick;

  @Override
  public void robotInit() {
    /**
     * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
     * 
     * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
     * first parameter
     * 
     * The motor type is passed as the second parameter. Motor type can either be:
     *  com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
     *  com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed
     * 
     * The example below initializes two brushless motors with CAN IDs 1 and 2. Change
     * these parameters to match your setup
     */
    leadMotor = new SparkMax(leadDeviceID, MotorType.kBrushless);
    followMotor = new SparkMax(followDeviceID, MotorType.kBrushless);
    /**
     * We modify the followMotor's SPARK MAX configurator to follow the leadMotor by
     * calling the follow() method and passing both the leader CAN ID and the invert value.
     * If only the leader CAN ID is passed, invert will default to false. 
     * 
     * NOTE: Changes made in the configurator do not get applied until the configurator is passed 
     * to the configure() method.
     */
    followMotorConfig = new SparkMaxConfig();
    followMotorConfig.follow(leadDeviceID, false);

    /**
     * The configure() method will apply the parameter changes to the motor its called from.
     * 
     * The first parameter is the configurator object which will contain all the changes
     * we want to apply.
     * 
     * The second parameter is the reset mode and can be either:
     *    kResetSafeParameters - Restore defaults before applying changes
     *    kNoResetSafeParameters - Don't restore defaults before applying changes
     * 
     * The third parameter is the persist mode and can be either:
     *    kPersistParameters -  Save settings onto SPARK MAX EEPROM
     *    kNoPersistParameters -  Don't save settings onto SPARK MAX EEPROM
     * 
     */
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    joystick = new Joystick(kJoystickPort);
  }

  @Override
  public void teleopPeriodic() {
    /**
     * followMotor will automatically follow whatever the applied output is on leadMotor.
     * 
     * Thus, set only needs to be called on leadMotor to control both of them
     */
    leadMotor.set(joystick.getY());
  }
}
