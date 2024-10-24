package frc.robot.subsystems.shooter;

import drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Shooter extends TalonFXSubsystem {

  public Shooter(TalonFXSubsystemConfiguration config) {
    super(config, ShooterGoal.IDLE);
  }

  public enum ShooterGoal implements TalonFXSubsystemGoal {
    IDLE(1250, ControlType.VELOCITY),
    EJECT(500, ControlType.VELOCITY),
    SHOOT(4000, ControlType.VELOCITY);

    public final double targetVelocity;
    public final ControlType controlType;

    private ShooterGoal(double velocity, ControlType controlType) {
      this.targetVelocity = velocity;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetVelocity;
    }
  }
}
