package frc.robot.field;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Distance FIELD_WIDTH = Inches.of(651.22);
  public static final Distance FIELD_HEIGHT = Inches.of(317.69);
  // Upper right is 0,0, blue-alliance orientation
  public static final Rectangle2d FIELD =
      new Rectangle2d(new Translation2d(0, 0), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));

  public static final Translation2d BLUE_HUB =
      new Translation2d(Inches.of(182.11), Inches.of(158.84));
  public static final Pose2d BLUE_HUB_POSE = new Pose2d(BLUE_HUB, Rotation2d.kZero);
  public static final Translation2d RED_HUB =
      new Translation2d(FIELD_WIDTH.minus(Inches.of(182.11)), Inches.of(158.84));
  public static final Pose2d RED_HUB_POSE = new Pose2d(RED_HUB, Rotation2d.kZero);
  public static final Rectangle2d BLUE_ZONE =
      new Rectangle2d(new Translation2d(0, 0), new Translation2d(Inches.of(156.61), FIELD_HEIGHT));
  public static final Rectangle2d RED_ZONE =
      new Rectangle2d(
          new Translation2d(FIELD_WIDTH, Inches.of(0)),
          new Translation2d(FIELD_WIDTH.minus(Inches.of(156.61)), FieldConstants.FIELD_HEIGHT));
  public static final Translation2d BLUE_OUTPOST_TRENCH =
      new Translation2d(Inches.of(156.61), Inches.of(100));
  public static final Translation2d BLUE_DEPOT_TRENCH =
      new Translation2d(Inches.of(156.61), Inches.of(FieldConstants.FIELD_HEIGHT.in(Inches) - 100));
  public static final Translation2d RED_DEPOT_TRENCH =
      new Translation2d(FIELD_WIDTH.minus(Inches.of(156.61)), Inches.of(100));
  public static final Translation2d RED_OUTPOST_TRENCH =
      new Translation2d(
          FIELD_WIDTH.minus(Inches.of(156.61)),
          Inches.of(FieldConstants.FIELD_HEIGHT.in(Inches) - 100));
}
