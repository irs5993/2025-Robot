// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismStates.ElevatorState;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX motor;
  private final MotionMagicVoltage controller;

  private ElevatorState currentState = ElevatorState.ZERO;
  private ElevatorState requestedState = this.currentState;
  private double desiredPosition = this.currentState.getValue();

  private ShuffleboardTab tab = Shuffleboard.getTab("Mechanisms");
  private GenericEntry readyEntry = tab.add("Elevator Ready", false).getEntry();
  private GenericEntry positionEntry = tab.add("Elevator Position", 0).getEntry();
  private GenericEntry desiredPositionEntry = tab.add("Elevator Desired Position", 0).getEntry();
  private GenericEntry currentStateEntry = tab.add("Current Elevator State", "").getEntry();

  public ElevatorSubsystem() {
    motor = new TalonFX(43, "canivore");
    motor.setPosition(0);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 1.5; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.03; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kG = 0.6;

    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motor.getConfigurator().apply(talonFXConfigs);

    controller = new MotionMagicVoltage(0);
  }

  @Override
  public void periodic() {
    setPosition(desiredPosition);

    if (atSetpoint()) {
      currentState = requestedState;
    }

    readyEntry.setBoolean(currentState == requestedState);
    positionEntry.setDouble(getPosition());
    desiredPositionEntry.setDouble(desiredPosition);
    currentStateEntry.setString(currentState.toString());
  }

  public void request(ElevatorState state) {
    desiredPosition = state.getValue();
    requestedState = state;
  }

  public void setPosition(double setpoint) {
    motor.setControl(controller.withPosition(setpoint).withLimitReverseMotion(getPosition() <= 0));
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public void resetPosition() {
    motor.setPosition(0);
  }

  public ElevatorState getState() {
    return currentState;
  }

  // Move this arbitrary value into constants
  public boolean atSetpoint() {
    return getError() <= 0.1;
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }

  public void stop() {
    motor.stopMotor();
  }

}
