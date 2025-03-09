// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.AlignState;
import frc.robot.Constants.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RobotHandler;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {

  private final VisionSubsystem visionSubsystem;
  private final CommandSwerveDrivetrain drivetrain;
  private final RobotHandler robotHandler;

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private Command pathFollowingCommand;
  private Pose2d targetPose;

  private RobotState armState;
  private AlignState alignState;

  private ShuffleboardTab tab = Shuffleboard.getTab("Align Reef");
  private GenericEntry distanceToTargetEntry = tab.add("Distance to Target", 0).getEntry();
  private GenericEntry alignStateEntry = tab.add("Align State", "").getEntry();
  private GenericEntry armStateEntry = tab.add("Arm State", "").getEntry();

  private EventScheduler scheduler = new EventScheduler();

  public AlignReef(CommandSwerveDrivetrain drivetrain, RobotHandler robotHandler, VisionSubsystem visionSubsystem,
      AlignState alignState) {
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;
    this.robotHandler = robotHandler;

    this.alignState = alignState;

    addRequirements(drivetrain, visionSubsystem);
  }

  @Override
  public void initialize() {
    switch (alignState) {
      case CORAL1_L2:
        armState = RobotState.CORAL_L2;
        targetPose = visionSubsystem.getCoral1Pose();
        break;
      case CORAL1_L3:
        armState = RobotState.CORAL_L3;
        targetPose = visionSubsystem.getCoral1Pose();
        break;
      case CORAL1_L4:
        armState = RobotState.CORAL_L4;
        targetPose = visionSubsystem.getCoral1Pose();
        break;
      case CORAL2_L2:
        armState = RobotState.CORAL_L2;
        targetPose = visionSubsystem.getCoral2Pose();
        break;
      case CORAL2_L3:
        armState = RobotState.CORAL_L3;
        targetPose = visionSubsystem.getCoral2Pose();
        break;
      case CORAL2_L4:
        armState = RobotState.CORAL_L4;
        targetPose = visionSubsystem.getCoral2Pose();
        break;
      case ALGEA_L1:
        armState = RobotState.ZERO;
        targetPose = visionSubsystem.getAlgeaPose();
        break;
      case ALGEA_L2:
        armState = RobotState.ZERO;
        targetPose = visionSubsystem.getAlgeaPose();
        break;
    }

    alignStateEntry.setString(alignState.toString());
    armStateEntry.setString(armState.toString());

    pathFollowingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        new PathConstraints(
            1.5, 1,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
        0.0);

    pathFollowingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distance = drivetrain.getState().Pose.getTranslation().getDistance(targetPose.getTranslation());
    System.out.println(distance);

    // distanceToTargetEntry.setDouble(distance);
    // if (distance <= 0.05) {
    // CommandScheduler.getInstance().cancel(pathFollowingCommand);
    // robotHandler.request(armState);
    // }

    // CommandScheduler.getInstance().schedule(
    // drivetrain.applyRequest(() -> drive.withVelocityX(maxSpeed * 0.3))
    // );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathFollowingCommand.cancel();
    // drivetrain.applyRequest(
    //     (() -> new SwerveRequest.SwerveDriveBrake()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
