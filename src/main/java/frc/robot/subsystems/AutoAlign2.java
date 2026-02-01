// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

import com.teamscreamrobotics.util.GeomUtil;
import com.teamscreamrobotics.util.Logger;

public class AutoAlign2 extends Command {

  private final RobotContainer container;
  private final CommandSwerveDrivetrain drivetrain;

  private boolean shouldEnd;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(11.0, 0, 0.0, new Constraints(3.8, 4.0));
  private final ProfiledPIDController headingController =
      new ProfiledPIDController(11, 0, 0, new Constraints(Units.degreesToRadians(360.0), 8.0));

  private Supplier<Pose2d> targetPose;

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double headingErrorAbs = 0.0;

  private double driveTolerance = 0.01;
  private final double headingTolerance = Units.degreesToRadians(1.0);

  private double currentDistance = 0.0;

  private final double ffMinRadius = 0.1;
  private final double ffMaxRadius = 0.4;

  private Timer rotateTimer = new Timer();
  private double rotationWait = 0.0;



  public AutoAlign2(RobotContainer container, Supplier<Pose2d> pose, boolean shouldEnd) {
    this.container = container;
    this.drivetrain = container.getSubsystems().drivetrain();
    targetPose = pose;

    this.shouldEnd = shouldEnd;

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
    setName("AutoAlign2");
  }

  public AutoAlign2(
      RobotContainer container, Supplier<Pose2d> pose, boolean shouldEnd, double driveTolerance) {
    this(container, pose, shouldEnd);
    this.driveTolerance = driveTolerance;
  }

  @Override
  public void initialize() {
    rotateTimer.reset();
    rotateTimer.start();
    Pose2d currentPose = drivetrain.getEstimatedPose();
    Twist2d fieldVel = drivetrain.getFieldVelocity();
    Translation2d linearFieldVel = new Translation2d(fieldVel.dx, fieldVel.dy);
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -linearFieldVel
                .rotateBy(
                    targetPose
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    headingController.reset(currentPose.getRotation().getRadians(), fieldVel.dtheta);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    if (targetPose == null) {
      return;
    }
    Pose2d currentPose = drivetrain.getEstimatedPose();
    Pose2d targetPose = this.targetPose.get();

    driveController.setConstraints(
        new Constraints(0.1, 0.1));

    currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScalar =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScalar
            + driveController.calculate(driveErrorAbs, 0.0);

    if (currentDistance < driveTolerance) {
      driveVelocityScalar = 0.0;
    }

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        rotateTimer.get() > rotationWait
            ? headingController.getSetpoint().velocity * ffScalar
                + headingController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
            : headingController.getSetpoint().velocity * ffScalar
                + headingController.calculate(
                    currentPose.getRotation().getRadians(), currentPose.getRotation().getRadians());
    headingErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (headingErrorAbs < headingTolerance) {
      thetaVelocity = 0.0;
    }

    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();

    drivetrain.setControl(
        drivetrain
            .getHelper()
            .getApplyRobotSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveVelocity.getX(),
                    driveVelocity.getY(),
                    thetaVelocity,
                    currentPose.getRotation())));

    Logger.log("AutoAlign/TargetPose", targetPose);
    Logger.log("AutoAlign/driveErrABS", this.driveErrorAbs);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    rotateTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return targetPose == null || (shouldEnd && Math.abs(driveErrorAbs) < driveTolerance &&
     (Math.abs(drivetrain.getEstimatedPose().getRotation().getRadians() - targetPose.get().getRotation().getRadians()) < driveTolerance * 2)
     );
  }

  public boolean hasReachedGoal() {
    return Math.abs(currentDistance) < driveTolerance && headingController.atSetpoint();
  }

  public boolean hasReachedGoal(double distTolerance) {
    return Math.abs(currentDistance) < distTolerance && headingController.atSetpoint();
  }
}
