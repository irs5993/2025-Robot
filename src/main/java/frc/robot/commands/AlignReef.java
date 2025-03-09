// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlignState;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.RobotHandler;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.RollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {

  private final VisionSubsystem visionSubsystem;
  private final RobotHandler robotHandler;
  private final RollerSubsystem rollerSubsystem;

  private Command roughAlign;
  private Command shortDrive;

  private Pose2d targetPose;

  private AlignState alignState;
  private RobotState armState;

  private ShuffleboardTab tab = Shuffleboard.getTab("Align Reef");
  private GenericEntry alignStateEntry = tab.add("Align State", "").getEntry();

  public AlignReef(RobotHandler robotHandler, VisionSubsystem visionSubsystem, RollerSubsystem rollerSubsystem,
      AlignState alignState) {
    this.robotHandler = robotHandler;
    this.visionSubsystem = visionSubsystem;
    this.rollerSubsystem = rollerSubsystem;

    this.alignState = alignState;

    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    switch (alignState) {
      case CORAL1_L2:
        targetPose = visionSubsystem.getCoral1Pose();
        armState = RobotState.CORAL_L2;
        break;
      case CORAL1_L3:
        targetPose = visionSubsystem.getCoral1Pose();
        armState = RobotState.CORAL_L3;
        break;
      case CORAL1_L4:
        targetPose = visionSubsystem.getCoral1Pose();
        armState = RobotState.CORAL_L4;
        break;
      case CORAL2_L2:
        targetPose = visionSubsystem.getCoral2Pose();
        armState = RobotState.CORAL_L2;
        break;
      case CORAL2_L3:
        targetPose = visionSubsystem.getCoral2Pose();
        armState = RobotState.CORAL_L3;
        break;
      case CORAL2_L4:
        targetPose = visionSubsystem.getCoral2Pose();
        armState = RobotState.CORAL_L4;
        break;
      case ALGAE_L1:
        targetPose = visionSubsystem.getAlgeaPose();
        armState = RobotState.CORAL_L2;
        break;
      case ALGAE_L2:
        targetPose = visionSubsystem.getAlgeaPose();
        armState = RobotState.ZERO;
        break;
    }

    if (targetPose == null) {
      targetPose = AutoBuilder.getCurrentPose();
    }

    alignStateEntry.setString(alignState.toString());

    roughAlign = AutoBuilder.pathfindToPose(
        targetPose,
        new PathConstraints(
            1.5, 1,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0.0);

    roughAlign.schedule();

    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    // new Pose2d(AutoBuilder.getCurrentPose().getTranslation(), Rotation2d.kZero),
    // new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(0)));
    // PathPlannerPath path = new PathPlannerPath(waypoints, new
    // PathConstraints(1.5, 1, 0.5, 0.5), null,
    // new GoalEndState(0, targetPose.getRotation()));
    // path.preventFlipping = false;

    // shortDrive = AutoBuilder.followPath(path);
    // shortDrive.schedule();
  }

  @Override
  public void execute() {
    System.out.println("executing wooo");

    if (roughAlign.isFinished()) {
      robotHandler.request(armState).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (roughAlign != null) {
      roughAlign.cancel();
    }
    if (shortDrive != null) {
      shortDrive.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
