package frc.robot.subsystems.intake.tilt;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class IntakeTiltConstants {
  public static final int CAN_ID = 5;

  public static final int GEAR_RATIO = 9;
  public static final Angle EXTENDED_POSITION = Degrees.of(45);

  public static class Motor {
    public static final double KP = 2.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 1.;
    public static final double KG = .5;
    public static final double KV = 1;
    public static final double KA = 1;

    public static final int CURRENT_LIMIT = 60;
  }
}
