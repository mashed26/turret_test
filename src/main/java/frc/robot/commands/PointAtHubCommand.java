package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.Supplier;

/**
 * Command to point the turret at the hub center using field-relative positioning. This command
 * continuously tracks the hub and adjusts the turret angle based on the robot's current pose on the
 * field.
 */
public class PointAtHubCommand extends Command {
  private final TurretSubsystem turret;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Translation2d targetPosition;

  /**
   * Creates a new PointAtHubCommand that aims at the alliance hub center.
   *
   * @param turret The turret subsystem
   * @param robotPoseSupplier Supplier for the robot's current pose
   */
  public PointAtHubCommand(TurretSubsystem turret, Supplier<Pose2d> robotPoseSupplier) {
    this(turret, robotPoseSupplier, FieldConstants.Hub.topCenterPoint.toTranslation2d());
  }

  /**
   * Creates a new PointAtHubCommand that aims at a specific target position.
   *
   * @param turret The turret subsystem
   * @param robotPoseSupplier Supplier for the robot's current pose
   * @param targetPosition The target position on the field to aim at
   */
  public PointAtHubCommand(
      TurretSubsystem turret, Supplier<Pose2d> robotPoseSupplier, Translation2d targetPosition) {
    this.turret = turret;
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetPosition = targetPosition;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    // Get current robot pose
    Pose2d robotPose = robotPoseSupplier.get();
    Translation2d robotTranslation = robotPose.getTranslation();
    Rotation2d robotHeading = robotPose.getRotation();

    // Calculate vector from robot to target
    Translation2d toTarget = targetPosition.minus(robotTranslation);

    // Calculate the absolute field angle to the target
    Rotation2d absoluteAngleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());

    // Calculate the robot-relative angle by subtracting the robot's heading
    // This gives us the angle the turret needs to be at relative to the robot
    Rotation2d robotRelativeAngle = absoluteAngleToTarget.minus(robotHeading);

    // Convert to degrees and command the turret
    double targetAngleDegrees = robotRelativeAngle.getDegrees();

    // Use the trackAngleCommand's logic to continuously update
    double safeTarget = getSafeTargetAngle(targetAngleDegrees);
    turret.setSetpointMotionMagicPosition(safeTarget);
  }

  /**
   * Helper method to get a safe target angle (borrowed from TurretSubsytem logic). This ensures the
   * turret doesn't exceed its physical limits.
   */
  private double getSafeTargetAngle(double requestedAngle) {
    double current = turret.getAngle().getDegrees();

    // Normalize to [-180, 180]
    double delta = normalizeAngle(requestedAngle - current);

    // Two possible paths
    double pathCW = delta > 0 ? delta - 360 : delta;
    double pathCCW = delta < 0 ? delta + 360 : delta;

    double endCW = current + pathCW;
    double endCCW = current + pathCCW;

    boolean cwValid =
        isWithinLimits(
            endCW,
            frc.robot.subsystems.turret.TurretConstants.MIN_ROT_DEG,
            frc.robot.subsystems.turret.TurretConstants.MAX_ROT_DEG);
    boolean ccwValid =
        isWithinLimits(
            endCCW,
            frc.robot.subsystems.turret.TurretConstants.MIN_ROT_DEG,
            frc.robot.subsystems.turret.TurretConstants.MAX_ROT_DEG);

    double chosenDelta;

    if (cwValid && ccwValid) {
      chosenDelta = Math.abs(pathCW) < Math.abs(pathCCW) ? pathCW : pathCCW;
    } else if (cwValid) {
      chosenDelta = pathCW;
    } else if (ccwValid) {
      chosenDelta = pathCCW;
    } else {
      // No legal path — clamp to nearest limit
      return Math.max(
          frc.robot.subsystems.turret.TurretConstants.MIN_ROT_DEG,
          Math.min(requestedAngle, frc.robot.subsystems.turret.TurretConstants.MAX_ROT_DEG));
    }

    return Math.max(
        frc.robot.subsystems.turret.TurretConstants.MIN_ROT_DEG,
        Math.min(chosenDelta + current, frc.robot.subsystems.turret.TurretConstants.MAX_ROT_DEG));
  }

  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }

  private boolean isWithinLimits(double angle, double min, double max) {
    return angle >= min && angle <= max;
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs continuously until interrupted
  }
}
