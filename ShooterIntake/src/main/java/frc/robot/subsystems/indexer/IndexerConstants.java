package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.ControlType;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import java.util.function.DoubleSupplier;

public class IndexerConstants {

  public static final double STAGE1_REDUCTION = 3.0;
  public static final double STAGE2_REDUCTION = 3.0;

  public static final TalonFXSubsystemConfiguration STAGE1_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    STAGE1_CONFIG.name = "IndexerStage1";

    STAGE1_CONFIG.codeEnabled = true;
    STAGE1_CONFIG.logTelemetry = false;

    STAGE1_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);

    STAGE1_CONFIG.enableSupplyCurrentLimit = true;
    STAGE1_CONFIG.supplyCurrentLimit = 20;

    STAGE1_CONFIG.sensorToMechRatio = STAGE1_REDUCTION;
  }

  public static final TalonFXSubsystemConfiguration STAGE2_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    STAGE2_CONFIG.name = "IndexerStage2";

    STAGE2_CONFIG.codeEnabled = true;
    STAGE2_CONFIG.logTelemetry = false;

    STAGE2_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive);

    STAGE2_CONFIG.enableSupplyCurrentLimit = true;
    STAGE2_CONFIG.supplyCurrentLimit = 20;

    STAGE2_CONFIG.sensorToMechRatio = STAGE2_REDUCTION;
  }

  public enum IndexerGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(7.5, ControlType.VOLTAGE),
    FEED(9.0, ControlType.VOLTAGE),
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
}
