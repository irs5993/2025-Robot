// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {

  private final VisionSubsystem visionSubsystem;
  private final CommandSwerveDrivetrain drivetrain;

  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public AlignReef(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
    this.drivetrain = drivetrain;
    this.visionSubsystem = visionSubsystem;

    addRequirements(drivetrain, visionSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // int tagId = visionSubsystem.getTargetId();
    int tagId = 21;
    var getTagPose = layout.getTagPose(tagId);

    Pose2d tagPose;
    if (getTagPose.isPresent()) {
      tagPose = getTagPose.get().toPose2d();
    } else {
      return;
    }

    Translation2d slot1Translation = new Translation2d(0.2, 0.25);
    Translation2d slot2Translation = new Translation2d(0.2, -0.25);

    slot1Translation = slot1Translation.rotateBy(tagPose.getRotation());
    slot2Translation = slot2Translation.rotateBy(tagPose.getRotation());

    Transform2d slot1Transform = new Transform2d(slot1Translation, new Rotation2d());
    Transform2d slot2Transform = new Transform2d(slot2Translation, new Rotation2d());

    Pose2d slot1 = tagPose.transformBy(slot1Transform);
    Pose2d slot2 = tagPose.transformBy(slot2Transform);

    // drivetrain
    // .applyRequest(() -> driveFacing.withTargetDirection(new Rotation2d())
    // .withVelocityX(x *
    // maxSpeed)
    // .withVelocityY(y * maxSpeed));

    CommandScheduler.getInstance().schedule(
        AutoBuilder.pathfindToPose(
            new Pose2d(slot1.getX(), slot1.getY(), new Rotation2d(Math.PI)),
            new PathConstraints(
                2, 2,
                Units.degreesToRadians(540), Units.degreesToRadians(720)),
            0.0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
