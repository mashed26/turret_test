package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import com.teamscreamrobotics.sim.SimWrapper;
import com.teamscreamrobotics.util.SimUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretConstants {

  public static final double REDUCTION = 13.2;
  public static final double MIN_ROT_DEG = -360.0;
  public static final double MAX_ROT_DEG = 360.0;

  public static final boolean ENABLE_SOFTWARE_LIMIT = true;
  public static final double FORWARD_SOFTWARE_LIMIT = 0.95;
  public static final double BACKWARD_SOFTWARE_LIMIT = -0.95;

  public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60(1);
  public static final int CAN_ID = 8;
  public static final double kP = 6.9;
  public static final double kI = 0.0;
  public static final double kD = 0.05;
  public static final double kS = 0.5;
  public static final double kV = .1;
  public static final double kA = 0.0;
  public static final double MAX_VEL = 30.0; // rot/s
  public static final double MAX_ACCEL = 40.0;
  public static final boolean BRAKE_MODE = true;
  public static final boolean ENABLE_STATOR_LIMIT = true;
  public static final int STATOR_CURRENT_LIMIT = 40;
  public static final boolean ENABLE_SUPPLY_LIMIT = true;
  public static final int SUPPLY_CURRENT_LIMIT = 40;

  public static final int CAN_INNER_ID = 5;
  public static final int CAN_OUTER_ID = 4;

  public static final int GEAR_0_TOOTH_COUNT = 132;
  public static final int GEAR_1_TOOTH_COUNT = 25;
  public static final int GEAR_2_TOOTH_COUNT = 24;
  public static final double SLOPE =
      (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
          / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(30.0, 0.0, 0.0);

  public static final DCMotorSim SIM = SimUtil.createDCMotorSim(DC_MOTOR, REDUCTION, 0.01);

  public static final TalonFXSubsystemConfiguration TURRET_CONFIG =
      new TalonFXSubsystemConfiguration();

  public static final CANcoderConfiguration INNER_CANCODER_CONFIG = new CANcoderConfiguration();

  public static final CANcoderConfiguration OUTTER_CODER_CONFIG = new CANcoderConfiguration();

  public static final double INNER_ENCODER_RATIO = GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT;

  public static final double OUTER_ENCODER_RATIO = GEAR_2_TOOTH_COUNT / GEAR_0_TOOTH_COUNT;
  public static final double CRT_MATCH_TOLERANCE = 0.01;

  static {
    TURRET_CONFIG.name = "Turret";

    TURRET_CONFIG.codeEnabled = true;
    TURRET_CONFIG.logTelemetry = false;
    TURRET_CONFIG.debugMode = false;

    TURRET_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(MAX_VEL, MAX_ACCEL)),
            false,
            false);

    TURRET_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(CAN_ID), InvertedValue.Clockwise_Positive);

    // Set current limits
    TURRET_CONFIG.statorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    TURRET_CONFIG.enableStatorCurrentLimit = TurretConstants.ENABLE_STATOR_LIMIT;
    TURRET_CONFIG.supplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    TURRET_CONFIG.enableSupplyCurrentLimit = TurretConstants.ENABLE_SUPPLY_LIMIT;

    // Set brake mode
    TURRET_CONFIG.neutralMode = NeutralModeValue.Brake;

    // Set motor rotation limits
    TURRET_CONFIG.maxUnitsLimit = FORWARD_SOFTWARE_LIMIT;
    TURRET_CONFIG.maxUnitsLimit = BACKWARD_SOFTWARE_LIMIT;

    // Apply gear ratio
    TURRET_CONFIG.sensorToMechRatio = REDUCTION;

    TURRET_CONFIG.cruiseVelocity = MAX_VEL;
    TURRET_CONFIG.acceleration = MAX_ACCEL;
    TURRET_CONFIG.slot0 =
        new ScreamPIDConstants(kP, kI, kD)
            .getSlot0Configs(new FeedforwardConstants(kV, kS, 0.0, kA));

    INNER_CANCODER_CONFIG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    INNER_CANCODER_CONFIG.MagnetSensor.withMagnetOffset(-0.00537109375);
    INNER_CANCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    OUTTER_CODER_CONFIG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    OUTTER_CODER_CONFIG.MagnetSensor.withMagnetOffset(-0.924072265625);
    OUTTER_CODER_CONFIG.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
  }
}
