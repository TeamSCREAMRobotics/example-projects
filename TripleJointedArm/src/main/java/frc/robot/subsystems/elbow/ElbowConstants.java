package frc.robot.subsystems.elbow;

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

public class ElbowConstants {

  public static final double REDUCTION = 24.0;

  public static final Length LENGTH = Length.fromInches(36);

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getKrakenX60(2), REDUCTION, 0.01);

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(300.0, 0, 0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elbow";
    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, false);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(9), InvertedValue.Clockwise_Positive);

    CONFIGURATION.sensorToMechRatio = REDUCTION;
  }
}
