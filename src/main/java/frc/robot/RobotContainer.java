// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.MechanismStates.ElbowState;
import frc.robot.Constants.MechanismStates.ElevatorState;
import frc.robot.Constants.MechanismStates.WristState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotHandler;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.WristSubsystem;

// #region Container
public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
                        Constants.OperatorConstants.ps5ControllerPort);
        private final CommandJoystick flightPad = new CommandJoystick(Constants.OperatorConstants.flightPadPort);

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        private final WristSubsystem wristSubsystem = new WristSubsystem();
        private final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
        private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
        private final RobotHandler robotHandler = new RobotHandler(elevatorSubsystem, elbowSubsystem, wristSubsystem,
                        ps5Controller);

        public RobotContainer() {
                configureBindings();
        }

        // #region Button Bindings
        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-ps5Controller.getLeftY() *
                                                MaxSpeed) // Drive
                                                // forward
                                                // with
                                                // negative
                                                // Y
                                                // (forward)
                                                .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed) // Drive left with
                                                // negative X
                                                // (left)
                                                .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate) // Drive
                                // counterclockwise
                                // with
                                // negative
                                // X
                                // (left)
                                ));

                // ps5Controller.cross().whileTrue(drivetrain.applyRequest(() -> brake));
                ps5Controller.povUp().whileTrue(drivetrain.applyRequest(
                                () -> driveFacing.withTargetDirection(new Rotation2d(0))
                                                .withVelocityX(-ps5Controller.getLeftY() *
                                                                MaxSpeed)
                                                .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed)));

                ps5Controller.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                flightPad.povRight().onTrue(
                                new InstantCommand(() -> wristSubsystem.request(WristState.MAX), wristSubsystem));
                flightPad.povUp().onTrue(
                                new InstantCommand(() -> wristSubsystem.request(WristState.S2), wristSubsystem));
                flightPad.povLeft().onTrue(
                                new InstantCommand(() -> wristSubsystem.request(WristState.S1), wristSubsystem));
                flightPad.povDown().onTrue(
                                new InstantCommand(() -> wristSubsystem.request(WristState.ZERO), wristSubsystem));

                flightPad.button(5).onTrue(
                                new InstantCommand(() -> elbowSubsystem.request(ElbowState.ZERO), elbowSubsystem));
                flightPad.button(3).onTrue(
                                new InstantCommand(() -> elbowSubsystem.request(ElbowState.S1), elbowSubsystem));
                flightPad.button(4).onTrue(
                                new InstantCommand(() -> elbowSubsystem.request(ElbowState.S2), elbowSubsystem));
                flightPad.button(6).onTrue(
                                new InstantCommand(() -> elbowSubsystem.request(ElbowState.MAX), elbowSubsystem));

                new Trigger(() -> flightPad.getRawAxis(2) > 0)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.ZERO),
                                                elevatorSubsystem));
                flightPad.button(1)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.S1),
                                                elevatorSubsystem));
                flightPad.button(2)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.S2),
                                                elevatorSubsystem));
                new Trigger(() -> flightPad.getRawAxis(3) > 0)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.MAX),
                                                elevatorSubsystem));

                ps5Controller.R2().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(2), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));
                ps5Controller.L2().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(-2), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));

                ps5Controller.triangle().onTrue(
                                robotHandler.request(RobotState.ASLANMAX));
                ps5Controller.square().onTrue(
                                robotHandler.request(RobotState.ORTAHALLI));
                ps5Controller.cross().onTrue(
                                robotHandler.request(RobotState.DEFAULT));
                ps5Controller.circle().onTrue(robotHandler.request(RobotState.AAAAAAAAAA));
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
