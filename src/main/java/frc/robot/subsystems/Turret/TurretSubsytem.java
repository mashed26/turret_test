package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Pivot subsystem using TalonFX with Krakenx60 motor */
@Logged(name = "TurretSubsystem")
public class TurretSubsytem extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final int canID = 8;
  private final double gearRatio = 13.2;
  private final double kP = 2;
  private final double kI = .1;
  private final double kD = 0;
  private final double kS = 0;
  private final double kV = .1;
  private final double kA = 0;
  private final double maxVelocity = 10; // rad/s
  private final boolean brakeMode = true;
  private final boolean enableStatorLimit = true;
  private final double statorCurrentLimit = 40;
  private final boolean enableSupplyLimit = false;
  private final double supplyCurrentLimit = 40;

  private final static double minRotDeg = -360;
  private final static double maxRotDeg = 360;

  // Motor controller
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /** Creates a new Pivot Subsystem. */
  public TurretSubsytem() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    
    // Set motor rotation limits
    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(.95);
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-.95);
    config.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

    // Apply gear ratio
    config.Feedback.SensorToMechanismRatio = gearRatio;

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Reset encoder position
    motor.setPosition(0);

    // Initialize simulation
    pivotSim =
        new SingleJointedArmSim(
            dcMotor, // Motor type
            gearRatio,
            0.01, // Arm moment of inertia - Small value since there are no arm parameters
            0.1, // Arm length (m) - Small value since there are no arm parameters
            Units.degreesToRadians(-90), // Min angle (rad)
            Units.degreesToRadians(90), // Max angle (rad)
            false, // Simulate gravity - Disable gravity for pivot
            Units.degreesToRadians(0) // Starting position (rad)
            );
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltageSignal, statorCurrentSignal, temperatureSignal);
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

    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(Rotations);
    double motorVelocity =
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec() * gearRatio).in(RotationsPerSecond);

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
    setVelocity(velocityDegPerSec, 0);
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
    return angle >= minRotDeg && angle <= maxRotDeg;
}

private double getSafeTargetAngle(double requestedAngle) {
    double current = getPositionDegrees();

    // shortest circular difference
    double delta = normalizeAngle(requestedAngle - current);

    // two possible paths
    double pathCW  = delta > 0 ? delta - 360 : delta;
    double pathCCW = delta < 0 ? delta + 360 : delta;

    double endCW  = current + pathCW;
    double endCCW = current + pathCCW;

    boolean cwValid  = isWithinLimits(endCW);
    boolean ccwValid = isWithinLimits(endCCW);

    double chosenDelta;

    if (cwValid && ccwValid) {
        chosenDelta = Math.abs(pathCW) < Math.abs(pathCCW)
                ? pathCW
                : pathCCW;
    } else if (cwValid) {
        chosenDelta = pathCW;
    } else if (ccwValid) {
        chosenDelta = pathCCW;
    } else {
        // No legal path — clamp to nearest limit
        return MathUtil.clamp(requestedAngle, minRotDeg, maxRotDeg);
    }

    return MathUtil.clamp(
        current + chosenDelta,
        minRotDeg,
        maxRotDeg
    );
}

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> {
          System.out.println("I AM RUNNING");
          System.out.println(angleDegrees);
          double safeTarget = getSafeTargetAngle(angleDegrees);
          System.out.println(safeTarget);

          double currentAngle = getPositionDegrees();
          double error = safeTarget - currentAngle;

          double velocityDegPerSec =
              Math.signum(error)
                  * Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
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
}
