package frc.robot.subsystems.indexer;

import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerGoal;

public class IndexerStageOne extends TalonFXSubsystem {

  private static final DigitalInput beam = new DigitalInput(0);

  public IndexerStageOne(TalonFXSubsystemConfiguration config) {
    super(config, IndexerGoal.IDLE);
  }

  public boolean beamTriggered() {
    return beam.get();
  }
}
