package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConstants;

public class IntakeConstants {

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Intake";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = false;

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(8), InvertedValue.Clockwise_Positive);

    SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
    SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 20;
  }
}
