package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConstants;

public class IndexerConstants {

  public static final TalonFXSubsystemConstants STAGE_1_CONSTANTS = new TalonFXSubsystemConstants();

  static {
    STAGE_1_CONSTANTS.name = "IndexerStage1";

    STAGE_1_CONSTANTS.codeEnabled = true;
    STAGE_1_CONSTANTS.logTelemetry = false;

    STAGE_1_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(9), InvertedValue.Clockwise_Positive);

    STAGE_1_CONSTANTS.enableSupplyCurrentLimit = true;
    STAGE_1_CONSTANTS.supplyCurrentLimit = 20;
  }

  public static final TalonFXSubsystemConstants STAGE_2_CONSTANTS = STAGE_1_CONSTANTS;

  static {
    STAGE_2_CONSTANTS.name = "IndexerStage2";

    STAGE_2_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);
  }
}
