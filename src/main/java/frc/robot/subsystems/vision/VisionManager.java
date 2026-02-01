// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Dashboard;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.teamscreamrobotics.util.Logger;
import com.teamscreamrobotics.vision.LimelightHelpers;
import com.teamscreamrobotics.vision.LimelightHelpers.PoseEstimate;
import com.teamscreamrobotics.vision.LimelightVision.Limelight;

/** Add your docs here. */
public class VisionManager {

  public static class Limelights {
    public static final Limelight FRONT_LEFT =
        new Limelight(
            "limelight-left",
            new Pose3d(
                -0.262158,
                0.313410,
                0.098568,
                new Rotation3d(Units.degreesToRadians(180), -Units.degreesToRadians(15.0), -Units.degreesToRadians(10.0))));
    public static final Limelight FRONT_RIGHT =
        new Limelight(
            "limelight-right",
            new Pose3d(
                0.262158,
                0.313410,
                0.098568,
                new Rotation3d(Units.degreesToRadians(180), -Units.degreesToRadians(15.0), Units.degreesToRadians(10))));
  }

  private final CommandSwerveDrivetrain drivetrain;

  private enum VisionType {
    REJECTED_INVALID,
    REJECTED_AMBIGUITY,
    REJECTED_MOVEMENT,
    MT,
    MT2
  }

  private final Limelight[] limelights =
      new Limelight[] {Limelights.FRONT_RIGHT, Limelights.FRONT_RIGHT};

  public VisionManager(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    LimelightHelpers.SetFiducialIDFiltersOverride(
        Limelights.FRONT_RIGHT.name(), FieldConstants.Hub.HUB_TAGS);
    LimelightHelpers.SetFiducialIDFiltersOverride(
        Limelights.FRONT_RIGHT.name(), FieldConstants.Hub.HUB_TAGS);
  }

  private boolean rejectEstimate(PoseEstimate estimate, Limelight limelight) {
    if (estimate == null
        || estimate.tagCount == 0
        || !FieldConstants.fieldArea.contains(estimate.pose)) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_INVALID);
      return true;
    } else if ((estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.3)
        && !Dashboard.disableAmbiguityRejection.get()) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_AMBIGUITY);
      return true;
    } else if ((Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
            > 540)
        || (drivetrain.getLinearVelocity().getNorm() > 3.5)) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_MOVEMENT);
      return true;
    } else {
      return false;
    }
  }

  private void addGlobalPoseEstimate(Limelight limelight) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);
    // PoseEstimate mt2Estimate =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
    PoseEstimate mtEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());

    boolean shouldUseMt1 = false; // !hasEnabled || Dashboard.disableMegatag2.get();
    /* boolean shouldUseMt2 =
    hasEnabled && !rejectEstimate(mt2Estimate) && !Dashboard.disableMegatag2.get(); */

    if (!shouldUseMt1 && !Dashboard.disableAllVisionUpdates.get()) {
      if (!rejectEstimate(mtEstimate, limelight)) {
        /* double stdFactor = Math.pow(mtEstimate.avgTagDist, 2.4) / (mtEstimate.tagCount * 0.5);
        double xyStds =
            (DriverStation.isDisabled() ? 0.2 : VisionConstants.xyStdBaseline) * stdFactor;
        double thetaStds =
            (DriverStation.isDisabled() ? 0.2 : VisionConstants.thetaStdBaseline) * stdFactor; */
            double stdFactor = Math.pow(mtEstimate.avgTagDist, 2.4) / (mtEstimate.tagCount * 0.5);
            double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;
            double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;
        drivetrain.addVisionMeasurement(
            mtEstimate.pose,
            mtEstimate.timestampSeconds,
            VecBuilder.fill(xyStds, xyStds, thetaStds),
            false);
        Logger.log("Vision/" + limelight.name() + "/PoseEstimate", mtEstimate.pose);
        Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT);
        Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
        Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
      } else {
        Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
        Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
        Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
      }
    }

    /* if (shouldUseMt2 && !Dashboard.disableAllVisionUpdates.get()) {
      double stdFactor = Math.pow(mt2Estimate.avgTagDist, 2.75) / (mt2Estimate.tagCount * 0.5);
      double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;
      double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;
      Pose2d combinedPose =
          new Pose2d(mt2Estimate.pose.getTranslation(), mtEstimate.pose.getRotation());
      drivetrain.addVisionMeasurement(
          mt2Estimate.pose,
          mt2Estimate.timestampSeconds,
          VecBuilder.fill(xyStds, xyStds, 999999999999.0),
          true);

        Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT2);
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", combinedPose, 1.5);
      Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
    } else {
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
      Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
    } */
  }

  public void periodic() {
    for (Limelight ll : limelights) {
      addGlobalPoseEstimate(ll);
    }
  }

}
