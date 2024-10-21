package frc.robot.subsystems.shooter;

import drivers.TalonFXSubsystem;

public class Shooter extends TalonFXSubsystem {

  public Shooter(TalonFXSubsystemConstants constants) {
    super(constants, TalonFXSubsystem.defaultGoal);
  }
}
