// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.AlignState;
import frc.robot.Constants.RobotState;
import frc.robot.commands.AlignReef;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotHandler;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

// #region Container
public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.45).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10%
                                                                                                     // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        /* Path follower */
        // private final SendableChooser<Command> autoChooser;

        private final CommandPS5Controller ps5Controller = new CommandPS5Controller(
                        Constants.OperatorConstants.ps5ControllerPort);
        private final CommandJoystick flightPad = new CommandJoystick(Constants.OperatorConstants.flightPadPort);

        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
        private final WristSubsystem wristSubsystem = new WristSubsystem();
        private final ElbowSubsystem elbowSubsystem = new ElbowSubsystem();
        private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final RobotHandler robotHandler = new RobotHandler(elevatorSubsystem, elbowSubsystem, wristSubsystem,
                        ps5Controller);
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(visionSubsystem);

        public RobotContainer() {
                // autoChooser = AutoBuilder.buildAutoChooser("Tests");
                // SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        // #region Button Bindings
        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-ps5Controller.getLeftY() *
                                                MaxSpeed)
                                                .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate)

                                ));

                ps5Controller.L2()
                                .whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-ps5Controller.getLeftY() *
                                                MaxSpeed * 0.35)
                                                .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed * 0.35)
                                                .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate * 0.35)

                                ));

                // ps5Controller.cross().whileTrue(drivetrain.applyRequest(() -> brake));
                // ps5Controller.povUp().whileTrue(drivetrain.applyRequest(
                // () -> driveFacing.withTargetDirection(new Rotation2d(0))
                // .withVelocityX(-ps5Controller.getLeftY() *
                // MaxSpeed)
                // .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed)));

                ps5Controller.options().onTrue(drivetrain.runOnce(() -> {
                        drivetrain.resetPose(new Pose2d(0.5, 8.0519016 / 2 - 0.25, new Rotation2d()));
                        drivetrain.seedFieldCentric();
                }));

                drivetrain.registerTelemetry(logger::telemeterize);

                // flightPad.povRight().onTrue(
                // new InstantCommand(() -> wristSubsystem.request(WristState.MAX),
                // wristSubsystem));
                // flightPad.povUp().onTrue(
                // new InstantCommand(() -> wristSubsystem.request(WristState.S2),
                // wristSubsystem));
                // flightPad.povLeft().onTrue(
                // new InstantCommand(() -> wristSubsystem.request(WristState.S1),
                // wristSubsystem));
                // flightPad.povDown().onTrue(
                // new InstantCommand(() -> wristSubsystem.request(WristState.ZERO),
                // wristSubsystem));

                // flightPad.button(5).onTrue(
                // new InstantCommand(() -> elbowSubsystem.request(ElbowState.ZERO),
                // elbowSubsystem));
                // flightPad.button(3).onTrue(
                // new InstantCommand(() -> elbowSubsystem.request(ElbowState.S1),
                // elbowSubsystem));
                // flightPad.button(4).onTrue(
                // new InstantCommand(() -> elbowSubsystem.request(ElbowState.S2),
                // elbowSubsystem));
                // flightPad.button(6).onTrue(
                // new InstantCommand(() -> elbowSubsystem.request(ElbowState.MAX),
                // elbowSubsystem));

                // new Trigger(() -> flightPad.getRawAxis(2) > 0)
                // .onTrue(new InstantCommand(() ->
                // elevatorSubsystem.request(ElevatorState.ZERO),
                // elevatorSubsystem));
                // flightPad.button(1)
                // .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.S1),
                // elevatorSubsystem));
                // flightPad.button(2)
                // .onTrue(new InstantCommand(() -> elevatorSubsystem.request(ElevatorState.S2),
                // elevatorSubsystem));
                // new Trigger(() -> flightPad.getRawAxis(3) > 0)
                // .onTrue(new InstantCommand(() ->
                // elevatorSubsystem.request(ElevatorState.MAX),
                // elevatorSubsystem));

                ps5Controller.R1().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(3.5), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));
                ps5Controller.L1().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(-3.5), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));

                flightPad.button(10).onTrue(
                                robotHandler.request(RobotState.MAX));

                ps5Controller.cross().onTrue(
                                robotHandler.request(RobotState.ZERO));
                ps5Controller.square().onTrue(
                                robotHandler.request(RobotState.CORAL_L2));
                ps5Controller.triangle().onTrue(
                                robotHandler.request(RobotState.CORAL_L3));
                ps5Controller.circle().onTrue(robotHandler.request(RobotState.CORAL_L4));

                ps5Controller.povUp().whileTrue(
                                new AlignReef(drivetrain, robotHandler, visionSubsystem,
                                                AlignState.ALGEA_L1));

                // ps5Controller.povUp().whileTrue(
                // AutoBuilder.pathfindToPose(
                // visionSubsystem.getAlgeaPose(),
                // new PathConstraints(
                // 1.5, 1,
                // Units.degreesToRadians(360),
                // Units.degreesToRadians(540)),
                // 0.0)

                // );

                // ps5Controller.povDown().whileTrue(
                // AutoBuilder.pathfindToPose(
                // new Pose2d(5.321046 + 3.525, 8.05 / 2 - 0.70, new Rotation2d()),
                // new PathConstraints(
                // 2, 2,
                // Units.degreesToRadians(540),
                // Units.degreesToRadians(720)),
                // 0.0));

        }

        public Command getAutonomousCommand() {
                return new PathPlannerAuto("New Auto");

        }
}
