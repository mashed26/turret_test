// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionManager;
import java.util.function.Supplier;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(CommandSwerveDrivetrain drivetrain) {}

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Request for rotating to a specific angle
  /*
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MaxSpeed * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  */

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final TurretSubsystem turret = new TurretSubsystem(TurretConstants.TURRET_CONFIG);
  public final Vision vision = new Vision(() -> drivetrain.getState().Pose);

  @Getter private final Subsystems subsystems = new Subsystems(drivetrain);
  @Getter private final VisionManager visionManager = new VisionManager(drivetrain);

  private double targetDegrees = 0;

  public RobotContainer() {
    configureBindings();
  }

  private double getNearest45DegreeAngle(double currentDegrees) {
    double target;

    if (currentDegrees > -180 && currentDegrees < -90) {
      target = -135;
    } else if (currentDegrees > -90 && currentDegrees < 0) {
      target = -45;
    } else if (currentDegrees > 0 && currentDegrees < 90) {
      target = 45;
    } else if (currentDegrees > 90 && currentDegrees < 180) {
      target = 135;
    } else {
      target = 0;
    }

    return target;
  }

  private double capturedTargetAngle = 0;

  private void configureBindings() {
    // Default swerve drive command
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        Math.pow(joystick.getLeftY(), 2)
                            * Math.signum(-joystick.getLeftY())
                            * MaxSpeed)
                    .withVelocityY(
                        Math.pow(joystick.getLeftX(), 2)
                            * Math.signum(-joystick.getLeftX())
                            * MaxSpeed)
                    .withRotationalRate(-Math.pow(joystick.getRightX(), 3) * MaxAngularRate)));

    // Idle while disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Swerve control
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() ->
    //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    // ));

    // joystick
    //     .x()
    //     .whileTrue(
    //         new AutoAlign2(
    //                 this,
    //                 () ->
    //                     new Pose2d(
    //                         new Translation2d(1.005, 5.310),
    //                         new Rotation2d(Units.degreesToRadians(-180))),
    //                 true,
    //                 0.05)
    //             .andThen(
    //                 new AutoAlign2(
    //                     this,
    //                     () ->
    //                         new Pose2d(
    //                             new Translation2d(1.005, 4.600),
    //                             new Rotation2d(Units.degreesToRadians(-180))),
    //                     true)));
    // joystick
    //     .a()
    //     .whileTrue(
    //         new AutoAlign2(
    //             this,
    //             () ->
    //                 new Pose2d(
    //                     new Translation2d(1.018, 4.618),
    //                     new Rotation2d(Units.degreesToRadians(-180))),
    //             true));

    joystick.a().whileTrue(turret.pointAtHubCenter(() -> drivetrain.getEstimatedPose()));

    joystick
        .rightBumper()
        .whileTrue(
            turret.moveToAngleCommandFR(Rotation2d.kCW_90deg, () -> drivetrain.getHeading()));

    joystick
        .y()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      capturedTargetAngle =
                          getNearest45DegreeAngle(
                              drivetrain.getState().Pose.getRotation().getDegrees());
                      DrivetrainConstants.HEADING_CONTROLLER_PROFILED.reset(
                          drivetrain.getState().Pose.getRotation().getRadians());
                    })
                .andThen(
                    drivetrain.applyRequest(
                        () ->
                            drivetrain
                                .getHelper()
                                .getFacingAngleProfiled(
                                    new Translation2d(-joystick.getLeftX(), -joystick.getLeftY()),
                                    Rotation2d.fromDegrees(capturedTargetAngle),
                                    DrivetrainConstants.HEADING_CONTROLLER_PROFILED)))
                .beforeStarting(() -> drivetrain.resetHeadingController()));

    // joystick.rightTrigger().onTrue(turret.moveToAngleCommandFR(270));
    // joystick.leftTrigger().onTrue(turret.moveToAngleCommandFR(-270));

    joystick
        .rightBumper()
        .whileTrue(turret.applyVoltageCommand(() -> (-joystick.getRightY() * 12.0)));

    // POV controls for incremental angle adjustment
    joystick.povUp().onTrue(Commands.runOnce(() -> targetDegrees += 10));
    joystick.povDown().onTrue(Commands.runOnce(() -> targetDegrees -= 10));
    joystick
        .povLeft()
        .onTrue(turret.moveToAngleCommandRR(() -> Rotation2d.fromDegrees(targetDegrees)));
    joystick
        .povRight()
        .onTrue(turret.moveToAngleCommandRR(() -> Rotation2d.kZero)); // Reset to forward

    // SysId routines
    /*
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    */
    // joystick
    //     .start()
    //     .and(joystick.leftBumper())
    //     .onTrue(Commands.runOnce(() -> SignalLogger.start()));
    // joystick
    //     .start()
    //     .and(joystick.rightBumper())
    //     .onTrue(Commands.runOnce(() -> SignalLogger.stop()));
    joystick.back().and(joystick.y()).whileTrue(turret.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(turret.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(turret.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(turret.sysIdQuasistatic(Direction.kReverse));

    // Reset field-centric heading
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void periodic() {
    // Update vision subsystem with current turret angle
    vision.setTurretAngle(turret.getAngle());
    Supplier<Pose2d> robotPose = () -> drivetrain.getState().Pose;

    // SmartDashboard telemetry
    // SmartDashboard.putNumber("Turret/Position", turret.getPositionDegrees());
    // SmartDashboard.putNumber("Turret/Velocity", turret.getVelocity());
    // SmartDashboard.putNumber("Turret/Current", turret.getCurrent());
    // SmartDashboard.putNumber("Turret/Target", targetDegrees);
    //
    // SmartDashboard.putString("Vision/Info", vision.getTrackingInfo());
    // SmartDashboard.putNumber("Vision/DesiredAngle", vision.getDesiredAngle());
    // SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
    // SmartDashboard.putNumber("Vision/Distance", vision.getDistanceToTarget());
    //
    // SmartDashboard.putNumber("RobotHeading", robotPose.get().getRotation().getDegrees());
    // SmartDashboard.putNumber(
    //     "Snap/Nearest45", getNearest45DegreeAngle(robotPose.get().getRotation().getDegrees()));

    // Trajectory calculations
    // if (vision.hasTarget()) {
    //   Trajectory.configure()
    //       .setGamePiece(GamePiece.FUEL)
    //       .setInitialHeight(HEIGHT)
    //       .setShotVelocity(5)
    //       .setTargetDistance(vision.getDistanceToTarget())
    //       .setTargetHeight(Trajectory.HUB_HEIGHT);
    //
    //   SmartDashboard.putNumber("Trajectory/OptimalAngle", Trajectory.getOptimalAngle());
    // }

    Logger.log(
        "Turret Pose",
        new Pose3d(
            robotPose.get().getX(),
            robotPose.get().getY(),
            0.5,
            new Rotation3d(
                0,
                0,
                robotPose.get().getRotation().getRadians() + turret.getAngle().getRadians())));
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }

  public Command getAutonomousCommand() {
    // Example auto command: Track target for 5 seconds
    return turret
        .moveToAngleCommandRR(() -> vision.getDesiredAngle())
        .withTimeout(5.0)
        .andThen(Commands.runOnce(() -> turret.stop()));
  }
}
