package frc.robot.subsystems.intake;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends TalonFXSubsystem {

  public IntakeRollers(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public enum IntakeRollersGoal implements TalonFXSubsystemGoal {
    STOW(() -> 0.0),
    INTAKE(() -> 6.0),
    OUTTAKE(() -> -6.0);

    public final DoubleSupplier voltage;

    private IntakeRollersGoal(DoubleSupplier voltage) {
      this.voltage = voltage;
    }

    @Override
    public ControlType controlType() {
      return ControlType.VOLTAGE;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    @Override
    public DoubleSupplier target() {
      return voltage;
    }
  }
}
