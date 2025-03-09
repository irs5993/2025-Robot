// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Max;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistance extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final double distance;
  private SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private Pose2d initialPose;

  public DriveDistance(CommandSwerveDrivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPose = drivetrain.getState().Pose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("distance", initialPose.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()));
    drivetrain.setControl(request.withVelocityX(MaxSpeed * 0.15));

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
