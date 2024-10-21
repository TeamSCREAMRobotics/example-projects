package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConstants;

public class ShooterConstants {

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Intake";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = false;

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive);

    SUBSYSTEM_CONSTANTS.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive)
        };

    SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
    SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 20;
  }
}
