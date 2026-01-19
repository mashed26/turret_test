// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretSubsytem;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    // Request for rotating to a specific angle
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TurretSubsytem turret = new TurretSubsytem(() -> drivetrain.getState().Pose);
    public final Vision vision = new Vision(() -> drivetrain.getState().Pose);

    private final double HEIGHT = 0; // TODO: Find height of robot

    private double targetDegrees = 0;

    public RobotContainer() {
        configureBindings();
    }

    private double getNearest45DegreeAngle(double currentDegrees) {
        while (currentDegrees > 180) currentDegrees -= 360;
        while (currentDegrees < -180) currentDegrees += 360;
        
        double rounded = Math.round(currentDegrees / 45.0) * 45.0;
        
        while (rounded > 180) rounded -= 360;
        while (rounded < -180) rounded += 360;
        
        return rounded;
    }

    private Command snapTo45Degrees() {
        return Commands.runOnce(() -> {
            double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
            double targetAngle = getNearest45DegreeAngle(currentAngle);
            SmartDashboard.putNumber("Snap/TargetAngle", targetAngle);
        }).andThen(
            drivetrain.applyRequest(() -> 
                fieldCentricFacingAngle
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withTargetDirection(Rotation2d.fromDegrees(
                        getNearest45DegreeAngle(drivetrain.getState().Pose.getRotation().getDegrees())
                    ))
            )
        );
    }

    private void configureBindings() {
        // Default swerve drive command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Math.pow(joystick.getLeftY(), 2) * Math.signum(-joystick.getLeftY()) * MaxSpeed)
                    .withVelocityY(Math.pow(joystick.getLeftX(), 2) * Math.signum(-joystick.getLeftX()) * MaxSpeed)
                    .withRotationalRate(-Math.pow(joystick.getRightX(), 3) * MaxAngularRate)
            )
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Swerve control
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.y().whileTrue(snapTo45Degrees());

        joystick.rightTrigger().onTrue(turret.moveToAngleCommandFR(270));
        joystick.leftTrigger().onTrue(turret.moveToAngleCommandFR(-270));

        joystick.rightBumper().whileTrue(
            turret.manualControlCommand(() -> -joystick.getRightY())
        );

        // POV controls for incremental angle adjustment
        joystick.povUp().onTrue(Commands.runOnce(() -> targetDegrees += 10));
        joystick.povDown().onTrue(Commands.runOnce(() -> targetDegrees -= 10));
        joystick.povLeft().onTrue(turret.moveToAngleCommandRR(targetDegrees));
        joystick.povRight().onTrue(turret.moveToAngleCommandRR(0)); // Reset to forward

        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    Supplier<Pose2d> robotPose = () -> drivetrain.getState().Pose;

    public void periodic() {
        // Update vision subsystem with current turret angle
        vision.setTurretAngle(turret.getPositionDegrees());

        // SmartDashboard telemetry
        SmartDashboard.putNumber("Turret/Position", turret.getPositionDegrees());
        SmartDashboard.putNumber("Turret/Velocity", turret.getVelocity());
        SmartDashboard.putNumber("Turret/Current", turret.getCurrent());
        SmartDashboard.putNumber("Turret/Target", targetDegrees);
        
        SmartDashboard.putString("Vision/Info", vision.getTrackingInfo());
        SmartDashboard.putNumber("Vision/DesiredAngle", vision.getDesiredAngle());
        SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
        SmartDashboard.putNumber("Vision/Distance", vision.getDistanceToTarget());

        SmartDashboard.putNumber("RobotHeading", robotPose.get().getRotation().getDegrees());
        SmartDashboard.putNumber("Snap/Nearest45", getNearest45DegreeAngle(robotPose.get().getRotation().getDegrees()));

        // Trajectory calculations
        if (vision.hasTarget()) {
            Trajectory.configure()
                .setGamePiece(GamePiece.FUEL)
                .setInitialHeight(HEIGHT)
                .setShotVelocity(5)
                .setTargetDistance(vision.getDistanceToTarget())
                .setTargetHeight(Trajectory.HUB_HEIGHT);

            SmartDashboard.putNumber("Trajectory/OptimalAngle", Trajectory.getOptimalAngle());
        }
    }

    public Command getAutonomousCommand() {
        // Example auto command: Track target for 5 seconds
        return turret.trackAngleCommand(() -> vision.getDesiredAngle())
            .withTimeout(5.0)
            .andThen(turret.stopCommand());
    }
}