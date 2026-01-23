package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
    public static final double TURRET_REDUCTION = 13.2;
    public static final double MIN_ROT_DEG = -360.0;
    public static final double MAX_ROT_DEG = 360.0;

    public static final DCMotor dcMotor = DCMotor.getKrakenX60(1);
    public static final int canID = 8;
    public static final double gearRatio = 13.2;
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

    public static final double GEAR_0_TOOTH_COUNT = 132;
    public static final double GEAR_1_TOOTH_COUNT = 25;
    public static final double GEAR_2_TOOTH_COUNT = 24.0;
    public static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
        / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

    public static final TalonFXSubsystemConfiguration TURRET_CONFIG =
        new TalonFXSubsystemConfiguration();

    static {
        TURRET_CONFIG.name = "Turret";

        TURRET_CONFIG.codeEnabled = true;
        TURRET_CONFIG.logTelemetry = false;
        TURRET_CONFIG.debugMode = false;

        TURRET_CONFIG.masterConstants =
            new TalonFXConstants(new CANDevice(8), InvertedValue.CounterClockwise_Positive);

        TURRET_CONFIG.neutralMode = NeutralModeValue.Brake;
        TURRET_CONFIG.rotorToSensorRatio = TURRET_REDUCTION;
        TURRET_CONFIG.enableSupplyCurrentLimit = true;
        TURRET_CONFIG.supplyCurrentLimit = 40;
        TURRET_CONFIG.cruiseVelocity = 30.0;
        TURRET_CONFIG.acceleration = 30.0;

        TURRET_CONFIG.slot0 =
            // PID Values.
            new ScreamPIDConstants(4.5, 0, 0)
                //Does not apply since there is no gravity acting on turret.
                .getSlot0Configs(new FeedforwardConstants(0, 0, 0, 0, GravityTypeValue.Arm_Cosine));
        TURRET_CONFIG.positionThreshold = Units.degreesToRotations(3.0);
    }
}
