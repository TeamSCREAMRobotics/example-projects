package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import util.AllianceFlipUtil;
import util.RunnableUtil.RunOnce;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private RunOnce operatorPerspectiveApplier = new RunOnce();

  public final PhoenixSwerveHelper helper;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    helper =
        new PhoenixSwerveHelper(
            this::getPose,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    System.out.println("[Init] Drivetrain initialization complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode) {
    for (SwerveModule mod : getModules()) {
      mod.getDriveMotor().setNeutralMode(driveMode);
      mod.getSteerMotor().setNeutralMode(steerMode);
    }
  }

  public void resetHeading() {
    seedFieldRelative(new Pose2d(getPose().getTranslation(), AllianceFlipUtil.getFwdHeading()));
  }

  public SwerveModule[] getModules() {
    return m_modules;
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return getState().Speeds;
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  @Override
  public void periodic() {
    attemptToSetPerspective();
    if (getCurrentCommand() != null) {
      DogLog.log("RobotState/Subsystems/Drivetrain/ActiveCommand", getCurrentCommand().toString());
    }
    DogLog.log("RobotState/Pose", getPose());
    DogLog.log("RobotState/Subsystems/Drivetrain/MeasuredStates", getState().ModuleStates);
    DogLog.log("RobotState/Subsystems/Drivetrain/SetpointStates", getState().ModuleTargets);
  }

  public void attemptToSetPerspective() {
    operatorPerspectiveApplier.runOnceWhenTrueThenWhenChanged(
        () -> setOperatorPerspectiveForward(AllianceFlipUtil.getFwdHeading()),
        DriverStation.getAlliance().isPresent(),
        DriverStation.getAlliance().orElse(null));
  }
}
