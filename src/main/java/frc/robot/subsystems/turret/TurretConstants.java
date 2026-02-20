package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class TurretConstants {

  public static final int CAN_ID = -1;
  public static final Angle MAX_ANGLE = Degrees.of(90);
  public static final Angle MIN_ANGLE = Degrees.of(-90);
  public static final double GEAR_RATIO = 17 / 3;

  public static class Motor {
    public static final double KP = 1.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;

    public static final int CURRENT_LIMIT = 60;
  }
}
