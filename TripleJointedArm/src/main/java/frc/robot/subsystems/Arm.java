package frc.robot.subsystems;

import data.DataConversions;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderConstants;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import math.IKSolver;
import math.ScreamMath;

public class Arm {
  public final Shoulder shoulder = new Shoulder(ShoulderConstants.CONFIGURATION);
  public final Elbow elbow = new Elbow(ElbowConstants.CONFIGURATION);
  public final Wrist wrist = new Wrist(WristConstants.CONFIGURATION);

  public final IKSolver solver =
      new IKSolver(ShoulderConstants.LENGTH, ElbowConstants.LENGTH);

  public Command inverse(Translation2d target, Rotation2d wristAngle) {
    Rotation2d[] angles = solver.solve(target, true);

    return Commands.defer(
        () ->
            Commands.parallel(
                shoulder.run(() -> shoulder.setSetpointPosition(angles[0].getRotations())),
                elbow.run(() -> elbow.setSetpointPosition(angles[1].getRotations())),
                wrist.run(() -> wrist.setSetpointPosition(wristAngle.getRotations()))),
        Set.of(shoulder, elbow, wrist));
  }

  public Command inverse(DoubleSupplier x, DoubleSupplier y) {
    return Commands.defer(
        () -> {
          Supplier<Rotation2d[]> angles =
              () ->
                  solver.solve(new Translation2d(x.getAsDouble(), y.getAsDouble()), true);
          return Commands.parallel(
              shoulder.run(() -> shoulder.setSetpointPosition(angles.get()[0].getRotations())),
              elbow.run(() -> elbow.setSetpointPosition(angles.get()[1].getRotations())),
              Commands.run(
                  () -> {
                    DogLog.log("Shoulder Target", angles.get()[0].getDegrees());
                    DogLog.log("Elbow Target", angles.get()[1].getDegrees());
                  }));
        },
        Set.of(shoulder, elbow, wrist));
  }

  public Pose3d getTargetPoint(Translation2d target, Pose2d robotPose) {
    Translation3d translation =
        ScreamMath.rotatePoint(DataConversions.projectTo3d(target), robotPose.getRotation());
    return new Pose3d(
        new Translation3d(
            robotPose.getX() + translation.getX(),
            robotPose.getY() + translation.getY(),
            translation.getZ()),
        new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
  }
}
