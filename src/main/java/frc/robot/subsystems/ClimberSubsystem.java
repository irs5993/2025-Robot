// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX climberMotor;

  private final PositionVoltage positionVoltageController;
  private final DutyCycleOut dutyCycleOutController;

  public ClimberSubsystem() {
    climberMotor = new TalonFX(47, "canivore");
    climberMotor.setPosition(0); 

    positionVoltageController = new PositionVoltage(0);
    dutyCycleOutController = new DutyCycleOut(0);

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    talonFXConfigs.Slot0.kP = 1;
    talonFXConfigs.Slot0.kI = 0;
    talonFXConfigs.Slot0.kD = 0.03;

    climberMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {

  }

  public void setPosition(double setpoint) {

    climberMotor.setControl(positionVoltageController.withPosition(setpoint)); 

  }

  public void setSpeed(double speed) {
    climberMotor.setControl(dutyCycleOutController.withOutput(speed));
  }

  public void stop() {
    climberMotor.stopMotor();
  }
}
