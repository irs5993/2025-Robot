// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.AlignState;
import frc.robot.Constants.RobotState;
import frc.robot.commands.AlignReef;
import frc.robot.commands.DriveDistance;
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
                        .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final SendableChooser<Command> autoChooser;

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

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureAuto();
                configureBindings();
        }

        private void configureAuto() {

                // var exampleAutoCommand = new PathPlannerAuto("Test Auto");
                // exampleAutoCommand.event("raiseArm").onTrue(robotHandler.request(RobotState.CORAL_L2));

                new EventTrigger("raiseArm").onTrue(robotHandler.request(RobotState.CORAL_L2));
        }

        // #region Button Bindings
        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive.withVelocityX(-ps5Controller.getLeftY() *
                                                MaxSpeed)
                                                .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed)
                                                .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate)

                                ));

                // Snake mode
                // ps5Controller.L2()
                // .whileTrue(drivetrain.applyRequest(() ->
                // drive.withVelocityX(-ps5Controller.getLeftY() *
                // MaxSpeed * 0.35)
                // .withVelocityY(-ps5Controller.getLeftX() * MaxSpeed * 0.35)
                // .withRotationalRate(-ps5Controller.getRightX() * MaxAngularRate * 0.35)

                // ));

                // Odometry reset
                ps5Controller.options().onTrue(drivetrain.runOnce(() -> {
                        drivetrain.resetPose(new Pose2d(0.5, 8.0519016 / 2 - 0.25, new Rotation2d()));
                        drivetrain.seedFieldCentric();
                }));

                drivetrain.registerTelemetry(logger::telemeterize);

                // Roller motor
                ps5Controller.R1().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(2.5), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));
                ps5Controller.L1().whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(-3.5), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));

                // Arm setpoints
                ps5Controller.cross().onTrue(
                                robotHandler.request(RobotState.ZERO));
                ps5Controller.square().onTrue(
                                robotHandler.request(RobotState.CORAL_L2));
                ps5Controller.triangle().onTrue(
                                robotHandler.request(RobotState.CORAL_L3));
                ps5Controller.circle().onTrue(robotHandler.request(RobotState.CORAL_L4));

                // Flight Pad Setpoints
                // Arm setpoints
                flightPad.povDown().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityX(
                                                MaxSpeed * -0.05).withVelocityY(0)));
                flightPad.povLeft().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * +0.05).withVelocityX(0)));
                flightPad.povUp().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityX(
                                                MaxSpeed * 0.05).withVelocityY(0)));
                flightPad.povRight().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * -0.05).withVelocityX(0)));
                flightPad.povUpRight().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * -0.05).withVelocityX(MaxSpeed * 0.05)));
                flightPad.povUpLeft().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * 0.05).withVelocityX(MaxSpeed * 0.05)));
                flightPad.povDownLeft().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * 0.05).withVelocityX(MaxSpeed * -0.05)));
                flightPad.povDownRight().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * -0.05).withVelocityX(MaxSpeed * -0.05)));
                flightPad.button(2).onTrue(
                                robotHandler.request(RobotState.ALGAE_L1));

                // Scoring with pose aligner
                ps5Controller.R2().and(ps5Controller.square()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL1_L2));
                ps5Controller.R2().and(ps5Controller.triangle()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL1_L3));
                ps5Controller.R2().and(ps5Controller.circle()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL1_L4));

                ps5Controller.L2().and(ps5Controller.square()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL2_L2));
                ps5Controller.L2().and(ps5Controller.triangle()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL2_L3));
                ps5Controller.L2().and(ps5Controller.circle()).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.CORAL2_L4));

                ps5Controller.povUp().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityX(
                                                MaxSpeed * 0.05).withVelocityY(0)));
                ps5Controller.povDown().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityX(
                                                MaxSpeed * -0.05).withVelocityY(0)));
                ps5Controller.povRight().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * -0.05).withVelocityX(0)));
                ps5Controller.povLeft().whileTrue(
                                drivetrain.applyRequest(() -> robotCentric.withVelocityY(
                                                MaxSpeed * +0.05).withVelocityX(0)));
                flightPad.button(3).whileTrue(
                                new AlignReef(robotHandler, visionSubsystem, rollerSubsystem,
                                                AlignState.ALGAE_L1));
                flightPad.button(4).onTrue(
                                Commands.run(() -> rollerSubsystem.setVoltage(4.5),
                                                rollerSubsystem));
                flightPad.button(1).onTrue(
                                robotHandler.request(RobotState.MAX)
                );
                flightPad.button(2).whileTrue(
                                Commands.runEnd(() -> rollerSubsystem.setVoltage(-2.5), () -> rollerSubsystem.stop(),
                                                rollerSubsystem));


        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

        }
}
