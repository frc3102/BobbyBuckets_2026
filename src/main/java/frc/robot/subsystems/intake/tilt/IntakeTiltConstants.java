package frc.robot.subsystems.intake.tilt;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class IntakeTiltConstants {
  public static final int CAN_ID = 24;

  public static final int GEAR_RATIO = 180;
  public static final Angle EXTENDED_POSITION = Degrees.of(120);
  public static final double IN_POSITION = 7;
  // original: -24, tight: -33, -30 middle
  public static final double OUT_POSITION = -28;
  public static final double FEED_POSITION = -5;

  public static class Motor {
    public static final double KP = 4.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = .25;
    public static final double KG = 0;
    public static final double KV = .05;
    public static final double KA = .1;
    public static final double MM_CV = 500;
    public static final double MM_A = 300;
    public static final double MM_JERK = 300;

    public static final int CURRENT_LIMIT = 60;
  }
}
