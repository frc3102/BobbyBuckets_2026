package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autoaim extends SubsystemBase {

  private final AutoaimIO io;
  private final PoseSupplier poseSupplier;
  private final RotationSupplier turreRotationSupplier;
  public Autoaim(AutoaimIO io, PoseSupplier poseSupplier, RotationSupplier turreRotationSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.turreRotationSupplier = turreRotationSupplier;
  }



  @FunctionalInterface
  public static interface PoseSupplier {
    public Pose2d supply();
  }

  @FunctionalInterface
  public static interface RotationSupplier {
    public Rotation2d supply();
  }
}
