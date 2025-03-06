// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KeepRollerPosition extends Command {
  private final RollerSubsystem rollerSubsystem;
  private double setpoint;

  public KeepRollerPosition(RollerSubsystem rollerSubsystem) {
    this.rollerSubsystem = rollerSubsystem;
    this.setpoint = 0;

    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = rollerSubsystem.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollerSubsystem.setPosition(setpoint + 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
