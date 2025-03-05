// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.MechanismStates.ElbowState;
import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.WristState;

public class RobotHandler extends SubsystemBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final ElbowSubsystem elbowSubsystem;
  private final WristSubsystem wristSubsystem;
  private final CommandPS5Controller ps5Controller;

  private RobotState currentState = RobotState.DEFAULT;
  private RobotState requestedState = this.currentState;

  private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  private GenericEntry readyEntry = tab.add("Arm Ready", false).getEntry();
  private GenericEntry requestedStateEntry = tab.add("Requested Arm State", "").getEntry();
  private GenericEntry currentStateEntry = tab.add("Current Arm State", "").getEntry();

  public RobotHandler(ElevatorSubsystem elevatorSubsystem, ElbowSubsystem elbowSubsystem,
      WristSubsystem wristSubsystem, CommandPS5Controller ps5Controller) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elbowSubsystem = elbowSubsystem;
    this.wristSubsystem = wristSubsystem;

    this.ps5Controller = ps5Controller;
  }

  @Override
  public void periodic() {
    if (elevatorSubsystem.getState() == requestedState.getElevatorState()
        && elbowSubsystem.getState() == requestedState.getElbowState()
        && wristSubsystem.getState() == requestedState.getWristState()) {
      currentState = requestedState;
      readyEntry.setBoolean(true);
      ps5Controller.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      readyEntry.setBoolean(false);
    }

    requestedStateEntry.setString(requestedState.toString());
    currentStateEntry.setString(currentState.toString());
  }

  public Command request(RobotState state) {
    requestedState = state;

    ElevatorState currentElevatorState = elevatorSubsystem.getState();
    ElbowState currentElbowState = elbowSubsystem.getState();
    WristState currentWristState = wristSubsystem.getState();

    ElevatorState requestedElevatorState = state.getElevatorState();
    ElbowState requestedElbowState = state.getElbowState();
    WristState requestedWristState = state.getWristState();

    int counter = 0;

    // Count the number of mechanisms moving up
    if (requestedElevatorState.getValue() - currentElevatorState.getValue() > 0) {
      counter++;
    }

    if (requestedElbowState.getValue() - currentElbowState.getValue() > 0) {
      counter++;
    }

    if (requestedWristState.getValue() - currentWristState.getValue() > 0) {
      counter++;
    }

    // If the number of mechanisms moving up is greater or equal to 2, treat the arm
    // motion as moving up. Otherwise, treat it as moving down
    if (counter >= 2) {
      // Moving up

      return new InstantCommand(() -> elevatorSubsystem.request(requestedElevatorState),
          elevatorSubsystem)
          .andThen(Commands.waitSeconds(0.1))
          .andThen(new InstantCommand(
              () -> elbowSubsystem.request(requestedElbowState),
              elbowSubsystem))
          .andThen(Commands.waitSeconds(0.1))
          .andThen(new InstantCommand(
              () -> wristSubsystem.request(requestedWristState),
              wristSubsystem));

    } else {
      // Moving down

      return new InstantCommand(() -> wristSubsystem.request(requestedWristState),
          wristSubsystem)
          .andThen(Commands.waitSeconds(0.1))
          .andThen(new InstantCommand(
              () -> elbowSubsystem.request(requestedElbowState),
              elbowSubsystem))
          .andThen(Commands.waitSeconds(0.1))
          .andThen(new InstantCommand(
              () -> elevatorSubsystem.request(requestedElevatorState),
              elevatorSubsystem));
    }

  }
}
