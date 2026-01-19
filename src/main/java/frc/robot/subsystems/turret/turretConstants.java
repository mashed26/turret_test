// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class turretConstants {

    public static final double TURRET_REDUCTION = 13.2;
    public static final double MIN_ROT_DEG = -360.0;
    public static final double MAX_ROT_DEG = 360.0;

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
