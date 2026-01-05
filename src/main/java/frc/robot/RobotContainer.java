// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.07) // Add a 5% deadband to drive and 7% to rotation
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // joystick command configs
    // TODO: move joystick command to seperate command file
    private static final double DRIVE_DEADBAND = 0.03;
    private static final double ROT_DEADBAND = 0.03;
    private static final double kDrive = 5;
    private static final double kRot = 7;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double vx = logScale(-joystick.getLeftY(), DRIVE_DEADBAND, kDrive);
                double vy = logScale(-joystick.getLeftX(), DRIVE_DEADBAND, kDrive);
                double o = logScale(-joystick.getRightX(), ROT_DEADBAND, kRot);

                SmartDashboard.putNumber("VelocityX Setpoint", vx * MaxSpeed);
                SmartDashboard.putNumber("VelocityY Setpoint", vy * MaxSpeed);
                SmartDashboard.putNumber("Omega Setpoint", o * MaxAngularRate);

                return drive
                    .withVelocityX(vx * MaxSpeed)
                    .withVelocityY(vy * MaxSpeed)
                    .withRotationalRate(o * MaxAngularRate);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> {
                Pose2d start = drivetrain.getStartingPose();
                if (start == null) return drive.withVelocityX(0).withVelocityY(0);

                double[] vel = drivetrain.calculateDriveToPose(start);
                if (vel == null) return brake;

                SmartDashboard.putNumber("x", vel[0]);
                SmartDashboard.putNumber("y", vel[1]);
                SmartDashboard.putNumber("r", vel[2]);

                return drive.withVelocityX(vel[0] * MaxSpeed/2)
                            .withVelocityY(vel[1] * MaxSpeed/2)
                            .withRotationalRate(vel[2] * MaxAngularRate);
            })
        );


        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(
          new DriveToPoseCommand(drivetrain, drive, brake, -1, 0, 0, MaxSpeed, MaxAngularRate), // backward 0.5m
          new DriveToPoseCommand(drivetrain, drive, brake, 0, -1, 0, MaxSpeed, MaxAngularRate), // move right 0.5m
          new DriveToPoseCommand(drivetrain, drive, brake, 1, 0, 0, MaxSpeed, MaxAngularRate), // move forward 0.5m
          new DriveToPoseCommand(drivetrain, drive, brake, 0, 1, 0, MaxSpeed, MaxAngularRate) // move left 0.5m
        );

        // return new DriveToPoseCommand(drivetrain, drive, brake, 1, 0, 0, MaxSpeed, MaxAngularRate);

        // return new DriveDistance(drivetrain, 0.2, 2, MaxSpeed, MaxAngularRate);

        // return new SequentialCommandGroup(
        //   new InstantCommand(() -> driveDistance(1))  
        // );

        // return Commands.print("No autonomous command configured");
    }

    private static double logScale(double in, double deadband, double k) {
        if (Math.abs(in) < deadband) return 0.0;

        double sign = Math.signum(in);
        double a = (Math.abs(in) - deadband) / (1.0 - deadband);
        double scaled = Math.log1p(k * a) / Math.log1p(k);
        return sign * scaled;
    }
}
