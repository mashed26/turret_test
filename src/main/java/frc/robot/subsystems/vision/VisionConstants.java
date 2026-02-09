package frc.robot.subsystems.vision;

import com.teamscreamrobotics.data.Length;

public class VisionConstants {

  public static final int resolutionWidth = 1280;
  public static final int resolutionHeight = 960;

  public static final double xyStdBaseline = 0.93;
  public static final double thetaStdBaseline = 12.5;

  public static final double xyMt2StdFactor = 0.3;

  public static final double LIMELIGHT_MOUNT_ANGLE = 39.0;
  public static final double LIMELIGHT_HEIGHT = Length.fromInches(14.125).getMeters();
}
