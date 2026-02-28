package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class TurretConstants {

  public static final int CAN_ID = 15;
  public static final Angle MAX_ANGLE = Degrees.of(10);
  public static final Angle MIN_ANGLE = Degrees.of(-180);
  public static final double GEAR_RATIO = 17.0 / 3.0;

  public static class Position {
    public static final Distance X_CENTER = Inches.of(-10);
    public static final Distance Y_CENTER = Inches.of(0);
  }

  public static class Motor {
    public static final double KP = 4.8;
    public static final double KI = 0;
    public static final double KD = .1;
    public static final double KS = .25;
    public static final double KG = 0;
    public static final double KV = .12;
    public static final double KA = .01;
    public static final double MM_KV = 0;
    public static final double MM_KA = 6;
    public static final double MM_CV = 3;
    public static final double MM_JERK = 100;

    public static final int CURRENT_LIMIT = 60;
  }
}
