package frc.robot.subsystems.indexer;

import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerGoal;

public class IndexerStageTwo extends TalonFXSubsystem {

  private static final DigitalInput beam = new DigitalInput(1);

  public IndexerStageTwo(TalonFXSubsystemConfiguration config) {
    super(config, IndexerGoal.IDLE);
  }

  public boolean beamTriggered() {
    return beam.get();
  }
}
