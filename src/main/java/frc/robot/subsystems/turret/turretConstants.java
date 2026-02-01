package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretConstants {

    public static final double TURRET_REDUCTION = 13.2;
    public static final double MIN_ROT_DEG = -360.0;
    public static final double MAX_ROT_DEG = 360.0;

    public static final boolean ENABLE_SOFTWARE_LIMIT = true;
    public static final double FORWARD_SOFTWARE_LIMIT = 0.95;
    public static final double BACKWARD_SOFTWARE_LIMIT = -0.95;

    public static final DCMotor dcMotor = DCMotor.getKrakenX60(1);
    public static final int canID = 8;
    public static final double kP = 5.5;
    public static final double kI = 0.0;
    public static final double kD = 0.05;
    public static final double kS = 0;
    public static final double kV = .1;
    public static final double kA = 0.0;
    public static final double maxVelocity = 40; // rad/s
    public static final double maxAccel = 40;
    public static final boolean brakeMode = true;
    public static final boolean enableStatorLimit = true;
    public static final double statorCurrentLimit = 40;
    public static final boolean enableSupplyLimit = true;
    public static final double supplyCurrentLimit = 40;

    public static final double minRotDeg = -360;
    public static final double maxRotDeg = 360;

    public static final int CAN_INNER_ID = 5;
    public static final int CAN_OUTTER_ID = 4;

    public static final double GEAR_0_TOOTH_COUNT = 132;
    public static final double GEAR_1_TOOTH_COUNT = 25;
    public static final double GEAR_2_TOOTH_COUNT = 24.0;
    public static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
        / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

    public static final TalonFXSubsystemConfiguration TURRET_CONFIG =
        new TalonFXSubsystemConfiguration();

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
    public static final MotionMagicConfigs MAGIC_CONFIGS = new MotionMagicConfigs();

    public static final CANcoderConfiguration INNER_CODER_CONFIG = new CANcoderConfiguration();
    public static final CANcoderConfiguration OUTTER_CODER_CONFIG = new CANcoderConfiguration();

    static {
        TURRET_CONFIG.name = "Turret";

        TURRET_CONFIG.codeEnabled = true;
        TURRET_CONFIG.logTelemetry = false;
        TURRET_CONFIG.debugMode = false;

        // Configure PID for slot 0
    Slot0Configs slot0 = MOTOR_CONFIG.Slot0;
    slot0.kP = TurretConstants.kP;
    slot0.kI = TurretConstants.kI;
    slot0.kD = TurretConstants.kD;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = TurretConstants.kS;
    slot0.kV = TurretConstants.kV;
    slot0.kA = TurretConstants.kA;

    MAGIC_CONFIGS.MotionMagicAcceleration = 40.0;
    MAGIC_CONFIGS.MotionMagicCruiseVelocity = 30.0;

    // Set current limits
    CurrentLimitsConfigs currentLimits = MOTOR_CONFIG.CurrentLimits;
    currentLimits.StatorCurrentLimit = TurretConstants.statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = TurretConstants.enableStatorLimit;
    currentLimits.SupplyCurrentLimit = TurretConstants.supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = TurretConstants.enableSupplyLimit;

    // Set brake mode
    MOTOR_CONFIG.MotorOutput.NeutralMode = TurretConstants.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    
    // Set motor rotation limits
    MOTOR_CONFIG.SoftwareLimitSwitch.withForwardSoftLimitThreshold(.95);
    MOTOR_CONFIG.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-.95);
    MOTOR_CONFIG.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    MOTOR_CONFIG.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

    // Apply gear ratio
    MOTOR_CONFIG.Feedback.SensorToMechanismRatio = TURRET_REDUCTION;

    INNER_CODER_CONFIG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    INNER_CODER_CONFIG.MagnetSensor.withMagnetOffset(-0.15576171875);
    INNER_CODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    OUTTER_CODER_CONFIG.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    OUTTER_CODER_CONFIG.MagnetSensor.withMagnetOffset(-0.20849609375);
    OUTTER_CODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;


    }

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(50.0, 0.0, 50.0);
}
