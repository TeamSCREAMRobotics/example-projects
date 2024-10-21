package frc.robot.subsystems.indexer;

import drivers.TalonFXSubsystem;
import drivers.TalonFXSubsystem.ControlType;
import drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class Indexer {
  private final TalonFXSubsystem stageOne;
  private final TalonFXSubsystem stageTwo;

  private final DigitalInput stageOneBeam = new DigitalInput(0);
  private final DigitalInput stageTwoBeam = new DigitalInput(1);

  public Indexer(
      TalonFXSubsystemConstants stageOneConstants, TalonFXSubsystemConstants stageTwoConstants) {
    stageOne = new TalonFXSubsystem(stageOneConstants, IndexerGoal.IDLE);
    stageTwo = new TalonFXSubsystem(stageTwoConstants, IndexerGoal.IDLE);
  }

  public enum IndexerGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(7.5, ControlType.VOLTAGE),
    SHOOT(9.0, ControlType.VOLTAGE),
    EJECT(-10.0, ControlType.VOLTAGE);

    public final double voltage;
    public final ControlType controlType;

    private IndexerGoal(double voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> voltage;
    }
  }

  public Command applyGoal(TalonFXSubsystemGoal goal) {
    return Commands.parallel(stageOne.applyGoal(goal), stageTwo.applyGoal(goal));
  }

  public Command applyStageOneGoal(TalonFXSubsystemGoal goal) {
    return stageOne.applyGoal(goal);
  }

  public Command applyStageTwoGoal(TalonFXSubsystemGoal goal) {
    return stageOne.applyGoal(goal);
  }

  public boolean getBothStagesTriggered() {
    return getStageOneTriggered() && getStageTwoTriggered();
  }

  public boolean getStageOneTriggered() {
    return !stageOneBeam.get();
  }

  public boolean getStageTwoTriggered() {
    return !stageTwoBeam.get();
  }
}
