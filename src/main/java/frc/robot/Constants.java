// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Value;

import frc.robot.Constants.MechanismStates.ElbowState;
import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.WristState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int ps5ControllerPort = 0;
    public static final int flightPadPort = 1;
  }

  public static class Vision {
    public static final String CAMERA_LEFT = "Left";
    public static final String CAMERA_RIGHT = "Right";

  }

  public enum RobotState {
    ZERO(ElevatorState.ZERO, ElbowState.ZERO, WristState.ZERO),
    CORAL_L2(ElevatorState.L2, ElbowState.L2, WristState.L2),
    CORAL_L3(ElevatorState.L3, ElbowState.L3, WristState.L3),
    CORAL_L4(ElevatorState.L4, ElbowState.L4, WristState.L4),
    MAX(ElevatorState.MAX, ElbowState.MAX, WristState.MAX);

    private final ElevatorState elevatorState;
    private final ElbowState elbowState;
    private final WristState wristState;

    RobotState(ElevatorState elevatorState, ElbowState elbowState, WristState wristState) {
      this.elevatorState = elevatorState;
      this.elbowState = elbowState;
      this.wristState = wristState;
    }

    public ElevatorState getElevatorState() {
      return this.elevatorState;
    }

    public ElbowState getElbowState() {
      return this.elbowState;
    }

    public WristState getWristState() {
      return this.wristState;
    }
  }

  public static class MechanismStates {
    public enum ElbowState {
      ZERO(1.1),
      L2(6.3),
      L3(15.5),
      L4(21.8),
      MAX(23);

      private final double value;

      ElbowState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }

    public enum WristState {
      ZERO(0),
      L2(24),
      L3(24),
      L4(22.3),
      MAX(25);

      private final double value;

      WristState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }

    public enum ElevatorState {
      ZERO(0),
      L2(0),
      L3(0),
      L4(25),
      MAX(26);

      private final double value;

      ElevatorState(double value) {
        this.value = value;
      }

      public double getValue() {
        return value;
      }
    }
  }
}
