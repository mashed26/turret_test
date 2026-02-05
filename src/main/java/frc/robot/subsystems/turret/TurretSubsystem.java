package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.math.ScreamMath;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.SimConstants;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;

public class TurretSubsystem extends TalonFXSubsystem {
  private final CANcoder innerEncoder;
  private final CANcoder outerEncoder;

  private final Ligament turretTwo =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withDynamicAngle(() -> Rotation2d.fromDegrees(getAngle().getDegrees()));

  private final Ligament turretOne =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withStaticAngle(Rotation2d.fromDegrees(90));

  public final Mechanism turretMech =
      new Mechanism("Turret Mech", turretOne, turretTwo)
          .withStaticPosition(
              new Translation2d(
                  (SimConstants.MECH_WIDTH / 2.0) + Units.inchesToMeters(12.125),
                  Units.inchesToMeters(15)));

  public final Mechanism2d robotTest = new Mechanism2d(1, 1);

  public final MechanismRoot2d robotRoot = robotTest.getRoot(getName(), 0.5, 0.5);

  public final MechanismLigament2d turret = new MechanismLigament2d("turret", 0.4, 0.0);
  private final SysIdRoutine routine;

  /** Creates a new Pivot Subsystem. */
  public TurretSubsystem(TalonFXSubsystemConfiguration config) {
    super(config);

    innerEncoder = new CANcoder(TurretConstants.CAN_INNER_ID);
    outerEncoder = new CANcoder(TurretConstants.CAN_OUTER_ID);
    // Initialize motor controller
    robotRoot.append(turret);

    innerEncoder.getConfigurator().apply(TurretConstants.INNER_CANCODER_CONFIG);
    outerEncoder.getConfigurator().apply(TurretConstants.OUTTER_CODER_CONFIG);

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(7), // Use dynamic voltage of 7 V
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state ->
                    SignalLogger.writeString(
                        "SysIdTurret_State", state.toString())), // Default config is fine for most
            new SysIdRoutine.Mechanism(
                volts -> master.setVoltage(volts.in(Volts)), // Apply voltage to the motor
                null,
                this));
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      turret.setAngle(getAngle().getDegrees());

      SmartDashboard.putData("turret mech", robotTest);
    }

    SmartDashboard.putNumber("Turret Velocity", getVelocity());
    SmartDashboard.putNumber(
        "Turret Angle",
        calculateTurretAngleFromCANCoderDegrees(
            Units.rotationsToDegrees(innerEncoder.getAbsolutePosition().getValueAsDouble()),
            Units.rotationsToDegrees(outerEncoder.getAbsolutePosition().getValueAsDouble())));
    SmartDashboard.putNumber(
        "Turret Inner Encoder Angle",
        Units.rotationsToDegrees(innerEncoder.getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Turret Outer Encoder Angle",
        Units.rotationsToDegrees(outerEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  private double lastTurretAngle = 0.0;

  public double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
    double difference = e2 - e1;

    if (difference > 250) {
      difference -= 360;
    }
    if (difference < -250) {
      difference += 360;
    }
    difference *= TurretConstants.SLOPE;

    double e1Rotations =
        (difference * TurretConstants.GEAR_0_TOOTH_COUNT / TurretConstants.GEAR_1_TOOTH_COUNT)
            / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360.0 + e1)
            * (TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT);

    double rotation =
        (TurretConstants.GEAR_1_TOOTH_COUNT / TurretConstants.GEAR_0_TOOTH_COUNT) * 360.0;

    double a0 = turretAngle;
    double aPlus = turretAngle + rotation;
    double aMinus = turretAngle - rotation;

    double error0 = Math.abs(a0 - lastTurretAngle);
    double errorPlus = Math.abs(aPlus - lastTurretAngle);
    double errorMinus = Math.abs(aMinus - lastTurretAngle);

    if (errorPlus < error0 && errorPlus < errorMinus) {
      turretAngle = aPlus;
    } else if (errorMinus < error0) {
      turretAngle = aMinus;
    }
    lastTurretAngle = turretAngle;
    return turretAngle;
  }

  // Does the actual check to ensure that angle is within bounds
  // and will not damage any parts of the turret.
  private boolean isWithinLimits(double angle) {
    return angle >= TurretConstants.MIN_ROT_DEG && angle <= TurretConstants.MAX_ROT_DEG;
  }

  private Rotation2d getSafeTargetAngle(Rotation2d requestedAngle) {
    Rotation2d current = getAngle();
    double currentDegrees = current.getDegrees();

    // shortest circular difference
    double delta =
        Units.radiansToDegrees(MathUtil.angleModulus(requestedAngle.minus(current).getRadians()));

    // two possible paths
    double pathCW = delta > 0 ? delta - 360 : delta;
    double pathCCW = delta < 0 ? delta + 360 : delta;

    double endCW = currentDegrees + pathCW;
    double endCCW = currentDegrees + pathCCW;

    boolean cwValid = isWithinLimits(endCW);
    boolean ccwValid = isWithinLimits(endCCW);

    double chosenDelta;

    if (cwValid && ccwValid) {
      chosenDelta = Math.abs(pathCW) < Math.abs(pathCCW) ? pathCW : pathCCW;
    } else if (cwValid) {
      chosenDelta = pathCW;
    } else if (ccwValid) {
      chosenDelta = pathCCW;
    } else {
      // No legal path — clamp to nearest limit
      return Rotation2d.fromDegrees(
          MathUtil.clamp(
              requestedAngle.getDegrees(),
              TurretConstants.MIN_ROT_DEG,
              TurretConstants.MAX_ROT_DEG));
    }

    return Rotation2d.fromDegrees(
        MathUtil.clamp(
            chosenDelta + currentDegrees,
            TurretConstants.MIN_ROT_DEG,
            TurretConstants.MAX_ROT_DEG));
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angle The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  // This is the robot relative verion of this command.
  public Command moveToAngleCommandRR(Supplier<Rotation2d> angle) {
    return run(
        () -> {
          Rotation2d safeTarget = getSafeTargetAngle(angle.get());
          setSetpointMotionMagicPosition(safeTarget.getRotations());
        });
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angle The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  // This is the robot relative verion of this command.
  public Command moveToAngleCommandRR(Rotation2d angle) {
    return moveToAngleCommandRR(() -> angle);
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angle The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(
      Supplier<Rotation2d> angle, Supplier<Rotation2d> robotHeading) {
    return moveToAngleCommandRR(() -> (angle.get().minus(robotHeading.get())));
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angle The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(Rotation2d angle, Supplier<Rotation2d> robotHeading) {
    return moveToAngleCommandFR(() -> angle, robotHeading);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command pointAtFieldPosition(
      Supplier<Translation2d> targetPosition, Supplier<Pose2d> robotPose) {
    return moveToAngleCommandFR(
        () ->
            ScreamMath.calculateAngleToPoint(
                robotPose.get().getTranslation(), targetPosition.get()),
        () -> robotPose.get().getRotation());
    /* return run(() -> {
      // Get current robot pose
      Pose2d robotPose = this.robotPose.get();
      Translation2d robotTranslation = robotPose.getTranslation();
      Rotation2d robotHeading = robotPose.getRotation();

      // Calculate the absolute field angle to the target
      Rotation2d absoluteAngleToTarget = ScreamMath.calculateAngleToPoint(robotTranslation, targetPosition);

      // Calculate the robot-relative angle
      // This is the angle the turret needs to point relative to the robot's forward direction
      Rotation2d robotRelativeAngle = absoluteAngleToTarget.minus(robotHeading);

      // Get safe target and set angle
      double targetAngleDegrees = robotRelativeAngle.getDegrees();
      double safeTarget = getSafeTargetAngle(targetAngleDegrees);
      setAngle(safeTarget);
    })
    .withName("PointAtFieldPosition"); */
  }

  public Command pointAtFieldPosition(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
    return pointAtFieldPosition(() -> targetPosition, robotPose);
  }

  /**
   * Creates a command to point the turret at the hub center. Uses the hub center position from
   * FieldConstants.
   *
   * @return A command that points the turret at the hub center
   */
  public Command pointAtHubCenter(Supplier<Pose2d> robotPose) {
    return pointAtFieldPosition(
            () ->
                AllianceFlipUtil.get(FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
            robotPose)
        .withName("PointAtHubCenter");
  }
}
