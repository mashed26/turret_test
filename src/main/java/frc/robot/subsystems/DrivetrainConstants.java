package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.util.PPUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class DrivetrainConstants {
  public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double maxAngularSpeedRads = 8.0;
  public static final double maxAzimuthVelRads = Units.rotationsToRadians(10);

  public static final Length atPoseDistThreshold = Length.fromInches(2.0);

  public static final int numModules = 4;

  public static final ScreamPIDConstants headingCorrectionConstants =
      new ScreamPIDConstants(8.0, 0.0, 0.0);

  public static final ProfiledPIDController headingControllerProfiled =
      new ProfiledPIDController(7.0, 0, 0, new Constraints(7.0, Units.degreesToRadians(720.0 * 5)));

  public static final ScreamPIDConstants pathTranslationConstants =
      new ScreamPIDConstants(10.0, 0.0, 0.0);
  public static final ScreamPIDConstants pathRotationConstants =
      new ScreamPIDConstants(7.0, 0.0, 0.0);

  public static final ModuleConfig moduleConfig =
      new ModuleConfig(Units.inchesToMeters(2), maxSpeed, 1.4, DCMotor.getKrakenX60(1), 85, 1);

  public static final RobotConfig robotConfig =
      new RobotConfig(
          Units.lbsToKilograms(125),
          6.883,
          moduleConfig,
          TunerConstants.frontLeftPos,
          TunerConstants.frontRightPos,
          TunerConstants.backLeftPos,
          TunerConstants.backRightPos);

  public static final PathFollowingController pathFollowingController =
      new PPHolonomicDriveController(
          PPUtil.screamPIDConstantsToPPConstants(pathTranslationConstants),
          PPUtil.screamPIDConstantsToPPConstants(pathRotationConstants));

  public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS =
      new ScreamPIDConstants(15.0, 0.0, 0.0);
  public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS =
      new ScreamPIDConstants(7.0, 0.0, 0.0);

  public static final PathFollowingController PATH_FOLLOWING_CONTROLLER =
      new PPHolonomicDriveController(
          PPUtil.screamPIDConstantsToPPConstants(PATH_TRANSLATION_CONSTANTS),
          PPUtil.screamPIDConstantsToPPConstants(PATH_ROTATION_CONSTANTS));

  public static final ProfiledPIDController HEADING_CONTROLLER_PROFILED =
      new ProfiledPIDController(7.0, 0, 0, new Constraints(7, Units.degreesToRadians(720 * 5)));
}
