// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RollerSubsystem extends SubsystemBase {

  private final TalonFX motor;
  private final MotionMagicVoltage controller;
  // Color sensor

  public RollerSubsystem() {
    motor = new TalonFX(46, "canivore");
    motor.setPosition(0);

    controller = new MotionMagicVoltage(0);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 70;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    motor.getConfigurator().apply(talonFXConfigs);

  }

  public void periodic() {
  

  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setPosition(double setpoint) {
    motor.setControl(controller.withPosition(setpoint));
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    motor.setPosition(0);
  }

  public void stop() {
    motor.stopMotor();
  }

}
