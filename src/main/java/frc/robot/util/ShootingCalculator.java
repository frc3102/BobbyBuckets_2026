package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.FieldConstants;
import frc.robot.game.GameState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.junction.Logger;

public class ShootingCalculator {
  private PoseSupplier poseSupplier;
  private ChassisSpeedsSupplier chassisSpeedsSupplier;

  private double distance = 0.0;
  private boolean isMoving = false;
  private double leadAngle = 0.0;

  private GameState gameState;

  private static final double MOVING_THRESHOLD_MPS = 0.3;
  private static final double BASE_FLIGHT_TIME_SECONDS = 0.5;

  /** Flight time scaling factor per meter of distance */
  private static final double FLIGHT_TIME_PER_METER = 0.08;

  private double angleToTarget;

  private ShootingCalculator(PoseSupplier poseSupplier, GameState gameState) {
    this.poseSupplier = poseSupplier;
    this.gameState = gameState;
  }

  private static ShootingCalculator instance;

  public static void init(PoseSupplier poseSupplier, GameState gameState) {
    instance = new ShootingCalculator(poseSupplier, gameState);
  }

  public static ShootingCalculator getInstance() {
    if (instance == null) {
      throw new RuntimeException("ShootingCalculator never initialized");
    }
    return instance;
  }

  public static record TargetDelta(Angle angle, Distance distance) {}

  public TargetDelta update(Translation2d target) {
    var pose = poseSupplier.supply();
    double headingRad = pose.getRotation().getRadians();
    double cosH = Math.cos(headingRad);
    double sinH = Math.sin(headingRad);
    double shooterXOffset = SuperstructureConstants.SHOOTER_OFFSET_X.in(Meters);
    double shooterYOffset = SuperstructureConstants.SHOOTER_OFFSET_Y.in(Meters);
    double turretX = pose.getX() + shooterXOffset * cosH - shooterYOffset * sinH;
    double turretY = pose.getY() + shooterXOffset * sinH + shooterYOffset * cosH;

    double dx = target.getX() - turretX;
    double dy = target.getY() - turretY;

    distance = Math.sqrt(dx * dx + dy * dy);
    double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = pose.getRotation().getDegrees();

    angleToTarget = normalizeAngle(fieldAngle - robotHeading);
    Logger.recordOutput("ShootingCalculator/Target", target);
    Logger.recordOutput("ShootingCalculator/Distance", Meters.of(distance).in(Feet));
    Logger.recordOutput("ShootingCalculator/Angle", angleToTarget);
    return new TargetDelta(Degrees.of(angleToTarget), Meters.of(distance));
  }

  public Translation2d getHub() {
    if (gameState.getAlliance() == Alliance.Red) {
      return FieldConstants.RED_HUB;
    } else {
      return FieldConstants.BLUE_HUB;
    }
  }

  private double normalizeAngle(double deg) {
    return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  @FunctionalInterface
  public static interface PoseSupplier {
    public Pose2d supply();
  }

  @FunctionalInterface
  public static interface ChassisSpeedsSupplier {
    public ChassisSpeeds supply();
  }
}
