package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.signals.InvertedValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import pid.ScreamPIDConstants;
import sim.SimWrapper;
import util.SimUtil;

public class WristConstants {

  public static final double REDUCTION = 12.0;

  public static final Length LENGTH = Length.fromInches(12);

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getKrakenX60(2), REDUCTION, 0.01);

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(100.0, 0, 0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Wrist";
    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, false);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);

    CONFIGURATION.sensorToMechRatio = REDUCTION;
  }
}
