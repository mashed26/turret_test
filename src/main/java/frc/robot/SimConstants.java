package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class SimConstants {

  public static final double EDGE_WIDTH = Units.inchesToMeters(10);

  public static final double MECH_WIDTH = Units.inchesToMeters(26 + EDGE_WIDTH);
  public static final double MECH_HEIGHT = Units.inchesToMeters(42 + EDGE_WIDTH);

  public static final Mechanism2d MEASURED_MECHANISM =
      new Mechanism2d(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);
  public static final Mechanism2d SETPOINT_MECHANISM =
      new Mechanism2d(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);
}
