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

  public enum RobotState {
    DEFAULT(ElevatorState.ZERO, ElbowState.ZERO, WristState.ZERO),
    ASLANMAX(ElevatorState.MAX, ElbowState.MAX, WristState.MAX),
    ORTAHALLI(ElevatorState.S2, ElbowState.S2, WristState.ZERO),
    AAAAAAAAAA(ElevatorState.S1, ElbowState.ZERO, WristState.S2);

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
      S1(7),
      S2(13),
      MAX(20);

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
      S1(7),
      S2(13),
      MAX(20);

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
      S1(7),
      S2(13),
      MAX(20);

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
