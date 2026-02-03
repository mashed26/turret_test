package frc.robot.subsystems;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.util.PPUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
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
