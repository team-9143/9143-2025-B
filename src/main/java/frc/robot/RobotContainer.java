// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

import frc.robot.commands.EjectFirstPieceCommand;
import frc.robot.commands.EjectStackedPieceCommand;
import frc.robot.commands.RealignPieceCommand;
import frc.robot.commands.JogPieceCommand;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.KitBot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver_controller = new CommandXboxController(0);
    private final CommandXboxController operator_controller = new CommandXboxController(1);

    public final Swerve swerve = TunerConstants.createDrivetrain();
    public final KitBot kitbot = new KitBot();

    private final SendableChooser<Command> autoChooser;
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

    public RobotContainer() {

        NamedCommands.registerCommand("EjectFirstPieceCommand", new EjectFirstPieceCommand(kitbot));
        NamedCommands.registerCommand("EjectStackedPieceCommand", new EjectStackedPieceCommand(kitbot));
        NamedCommands.registerCommand("RealignPieceCommand", new RealignPieceCommand(kitbot));
        NamedCommands.registerCommand("JogPieceCommand", new JogPieceCommand(kitbot));

        // Create auto chooser and put it on the Auto tab in Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser("1m Forward");
        autoTab.add("Auto Mode", autoChooser)
            .withSize(3, 2)
            .withPosition(0, 0);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            swerve.applyRequest(() ->
                drive.withVelocityX(-driver_controller.getLeftY() * MaxSpeed * 0.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_controller.getLeftX() * MaxSpeed * 0.75) // Drive left with negative X (left)
                    .withRotationalRate(-driver_controller.getRightX() * MaxAngularRate)// Drive clockwise with X (right)
            )
        );

        driver_controller.a().whileTrue(swerve.applyRequest(() -> brake));
        driver_controller.b().whileTrue(swerve.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_controller.getLeftY(), -driver_controller.getLeftX()))
        ));

        driver_controller.povUp().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver_controller.povDown().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driver_controller.povLeft().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        driver_controller.povRight().whileTrue(swerve.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_controller.back().and(driver_controller.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        driver_controller.back().and(driver_controller.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        driver_controller.start().and(driver_controller.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        driver_controller.start().and(driver_controller.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        driver_controller.leftBumper().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        swerve.registerTelemetry(logger::telemeterize);

        // KitBot first piece eject
        operator_controller.b().onTrue(new EjectFirstPieceCommand(kitbot));

        // KitBot stacked piece eject
        operator_controller.y().onTrue(new EjectStackedPieceCommand(kitbot));

        // KitBot re-align
        operator_controller.x().whileTrue(new RealignPieceCommand(kitbot));

        // KitBot jog
        operator_controller.a().whileTrue(new JogPieceCommand(kitbot));
    }

    public Command getAutonomousCommand() {
        // Run the path selected from the auto chooser
        return autoChooser.getSelected();
    }

    public void disabledInit() {
        KitBot.rollerMotor.stopMotor();
    }
}