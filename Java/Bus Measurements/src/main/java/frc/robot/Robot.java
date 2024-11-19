/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to read basic bus measurements from the
 * SparkMax
 */
public class Robot extends TimedRobot {
  private SparkMax motor;
  private Joystick joystick;

  @Override
  public void robotInit() {
    // Initialize the SPARK MAX and joystick
    motor = new SparkMax(1, MotorType.kBrushless);
    joystick = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    /*
     * There are several useful bus measurements you can get from the SPARK MAX
     * by calling the SparkMax object's getter functions. This includes bus
     * voltage (V), output current (A), Applied Output (duty cycle), and motor
     * temperature (C).
     */
    double busVoltage = motor.getBusVoltage();
    double current = motor.getOutputCurrent();
    double appliedOut = motor.getAppliedOutput();
    double temperature = motor.getMotorTemperature();

    // Display values read from the SPARK MAX on SmartDashboard
    SmartDashboard.putNumber("Bus Voltage", busVoltage);
    SmartDashboard.putNumber("Current", current);
    SmartDashboard.putNumber("Applied Output", appliedOut);
    SmartDashboard.putNumber("Motor Temperature", temperature);
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Apply the joystick's y-axis value [-1, 1] to the motor to see how the bus
     * measurements change while the motor moves.
     */
    motor.set(joystick.getY());
  }
}
