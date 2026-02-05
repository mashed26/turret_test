package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.math.ScreamMath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.SimConstants;
import java.util.function.Supplier;

/** Pivot subsystem using TalonFX with Krakenx60 motor */
@Logged(name = "TurretSubsystem")
public class TurretSubsytem extends SubsystemBase {
  // Motor controller
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  private final CANcoder innerEncoder;
  private final CANcoder outerEncoder;

  // Simulation
  private final SingleJointedArmSim pivotSim;
  private Supplier<Pose2d> robotPose;

  private final Ligament turretTwo =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withDynamicAngle(() -> Rotation2d.fromDegrees(getPositionDegrees()));

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
  public TurretSubsytem(Supplier<Pose2d> robotPose) {
    this.robotPose = robotPose;

    // Initialize motor controller
    robotRoot.append(turret);

    motor = new TalonFX(TurretConstants.CAN_ID);
    innerEncoder = new CANcoder(TurretConstants.CAN_INNER_ID);
    outerEncoder = new CANcoder(TurretConstants.CAN_OUTTER_ID);

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    innerEncoder.getConfigurator().apply(TurretConstants.INNER_CODER_CONFIG);
    outerEncoder.getConfigurator().apply(TurretConstants.OUTTER_CODER_CONFIG);

    // Apply configuration
    motor.getConfigurator().apply(TurretConstants.MOTOR_CONFIG);
    motor.getConfigurator().apply(TurretConstants.MAGIC_CONFIGS);

    // Reset encoder position
    motor.setPosition(0);

    // Initialize simulation
    pivotSim =
        new SingleJointedArmSim(
            TurretConstants.DC_MOTOR, // Motor type
            TurretConstants.TURRET_REDUCTION,
            0.01, // Arm moment of inertia - Small value since there are no arm parameters
            0.1, // Arm length (m) - Small value since there are no arm parameters
            Units.degreesToRadians(-360), // Min angle (rad)
            Units.degreesToRadians(360), // Max angle (rad)
            false, // Simulate gravity - Disable gravity for pivot
            Units.degreesToRadians(0) // Starting position (rad)
            );

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
                volts -> motor.setControl(new VoltageOut(volts)), // Apply voltage to the motor
                null,
                this));
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      turret.setAngle(getPositionDegrees());

      SmartDashboard.putData("turret mech", robotTest);
    }

    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);

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
    SmartDashboard.putNumber("Turret Velocity", getVelocity());
  }

  /** Update simulation. */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    // pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()),
    // pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use motor voltage for TalonFX simulation input
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotSim.getCurrentDrawAmps()));

    double motorPosition =
        Radians.of(pivotSim.getAngleRads() * TurretConstants.TURRET_REDUCTION).in(Rotations);
    double motorVelocity =
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec() * TurretConstants.TURRET_REDUCTION)
            .in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the current position in Rotations.
   *
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current position in Degrees.
   *
   * @return Position in Degrees.
   */
  @Logged(name = "Position/Degrees")
  public double getPositionDegrees() {
    return getPosition() * 360;
  }

  /**
   * Get the current velocity in rotations per second.
   *
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   *
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   *
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   *
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set pivot angle.
   *
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set pivot angle with acceleration.
   *
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    // motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  /**
   * Set pivot angular velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, TurretConstants.MAX_ACCEL);
  }

  /**
   * Set pivot angular velocity with acceleration.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    // motor.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
    motor.setControl(velocityRequest.withVelocity(velocityRotations));
  }

  /**
   * Set motor voltage directly.
   *
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Get the pivot simulation for testing.
   *
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the pivot to a specific angle.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the pivot to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  private static double lastTurretAngle = 0.0;

  public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
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

  // Normalizes the input angle to the range of [-360, 360]
  // Allows to find shorter angles to travel faster.
  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
  }

  // Does the actual check to ensure that angle is within bounds
  // and will not damage any parts of the turret.
  private boolean isWithinLimits(double angle) {
    return angle >= TurretConstants.MIN_ROT_DEG && angle <= TurretConstants.MAX_ROT_DEG;
  }

  private double getSafeTargetAngle(double requestedAngle) {
    double current = getPositionDegrees();

    // shortest circular difference
    double delta = normalizeAngle(requestedAngle - current);

    // two possible paths
    double pathCW = delta > 0 ? delta - 360 : delta;
    double pathCCW = delta < 0 ? delta + 360 : delta;

    double endCW = current + pathCW;
    double endCCW = current + pathCCW;

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
      return MathUtil.clamp(
          requestedAngle, TurretConstants.MIN_ROT_DEG, TurretConstants.MAX_ROT_DEG);
    }

    return MathUtil.clamp(
        chosenDelta + current, TurretConstants.MIN_ROT_DEG, TurretConstants.MAX_ROT_DEG);
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  // This is the robot relative verion of this command.
  public Command moveToAngleCommandRR(double angleDegrees) {
    return run(() -> {
          // System.out.println(angleDegrees);
          double safeTarget = getSafeTargetAngle(angleDegrees);
          // System.out.println(safeTarget);

          double currentAngle = getPositionDegrees();
          double error = safeTarget - currentAngle;

          double velocityDegPerSec =
              Math.signum(error)
                  * Math.min(
                      Math.abs(error) * 2.0, Units.radiansToDegrees(TurretConstants.MAX_VEL));
          setVelocity(velocityDegPerSec);
        })
        .until(
            () -> {
              // Calculates the shortest safe path to get to the target.
              double safeTarget = getSafeTargetAngle(angleDegrees);
              double currentAngle = getPositionDegrees();

              return Math.abs(safeTarget - currentAngle) < 2.0; // 2 degree tolerance
            })
        .finallyDo(interrupted -> setVelocity(0));
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(Supplier<Rotation2d> angle) {
    return run(() -> {
          //  System.out.println(angleDegrees);
          double robotHeading = robotPose.get().getRotation().getDegrees();

          double safeTarget = getSafeTargetAngle(angle.get().getDegrees() + (robotHeading));
          // System.out.println(safeTarget);
          setAngle(safeTarget);

          /* double currentAngle = getPositionDegrees();
          double error = safeTarget - currentAngle;

          double velocityDegPerSec =
              Math.signum(error)
                  * Math.min(
                      Math.abs(error) * 7.0, Units.radiansToDegrees(TurretConstants.MAX_VEL));
          setVelocity(velocityDegPerSec); */
        });
        /* .until(
            () -> {
              // Calculates the shortest safe path to get to the target.
              // double robotHeading = robotPose.get().getRotation().getDegrees();
              double safeTarget = getSafeTargetAngle(angle.get().getDegrees());
              // double currentAngle = getPositionDegrees();
              //  System.out.println(safeTarget);

              return Math.abs(safeTarget) < 2.0; // 2 degree tolerance
            })
        .finallyDo(interrupted -> setVelocity(0)); */
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(Rotation2d angle) {
    return moveToAngleCommandFR(() -> angle);
  }

  /**
   * Creates a command to stop the pivot.
   *
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   *
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the pivot at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }

  public Command moveToAnglePositionCommand(double angleDegrees) {
    return runOnce(
            () -> {
              double safeTarget = getSafeTargetAngle(angleDegrees);
              setAngle(safeTarget);
            })
        .andThen(
            run(
                () -> {
                  // Keep checking if we've reached the target
                }))
        .until(
            () -> {
              double safeTarget = getSafeTargetAngle(angleDegrees);
              return Math.abs(safeTarget - getPositionDegrees()) < 2.0;
            })
        .withName("MoveToAnglePosition:" + angleDegrees);
  }

  public Command trackAngleCommand(java.util.function.DoubleSupplier angleSupplier) {
    return run(() -> {
          double targetAngle = angleSupplier.getAsDouble();
          double safeTarget = getSafeTargetAngle(targetAngle);
          setAngle(safeTarget);
        })
        .withName("TrackAngle");
  }

  public Command manualControlCommand(java.util.function.DoubleSupplier speedSupplier) {
    return run(() -> {
          double speed = speedSupplier.getAsDouble();
          setVelocity(speed * Units.radiansToDegrees(TurretConstants.MAX_VEL));
        })
        .withName("ManualControl");
  }

  public void stop() {
    motor.stopMotor();
  }

  public boolean atTarget(double targetDegrees) {
    double safeTarget = getSafeTargetAngle(targetDegrees);
    return Math.abs(safeTarget - getPositionDegrees()) < 2.0;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command pointAtFieldPosition(Translation2d targetPosition) {
    return moveToAngleCommandFR(() -> ScreamMath.calculateAngleToPoint(robotPose.get().getTranslation(), targetPosition));
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

/**
 * Creates a command to point the turret at the hub center.
 * Uses the hub center position from FieldConstants.
 *
 * @return A command that points the turret at the hub center
 */
public Command pointAtHubCenter() {
  return pointAtFieldPosition(
      frc.robot.constants.FieldConstants.Hub.hubCenter)
      .withName("PointAtHubCenter");
}

/**
 * Creates a command to point the turret at the opposing hub center.
 * Uses the opposing hub center position from FieldConstants.
 *
 * @return A command that points the turret at the opposing hub center
 */
public Command pointAtOpposingHubCenter() {
  return pointAtFieldPosition(
      frc.robot.constants.FieldConstants.Hub.oppHubCenter)
      .withName("PointAtOpposingHubCenter");
}

public double calculateAngleToFieldPosition(Translation2d targetPosition) {
  Pose2d robotPose = this.robotPose.get();
  Translation2d robotTranslation = robotPose.getTranslation();
  Rotation2d robotHeading = robotPose.getRotation();

  // Calculate vector from robot to target
  Translation2d toTarget = targetPosition.minus(robotTranslation);

  // Calculate the absolute field angle to the target
  Rotation2d absoluteAngleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());

  // Calculate the robot-relative angle
  Rotation2d robotRelativeAngle = absoluteAngleToTarget.minus(robotHeading);

  return robotRelativeAngle.getDegrees();
}
}
