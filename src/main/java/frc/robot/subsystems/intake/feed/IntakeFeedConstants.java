package frc.robot.subsystems.intake.feed;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class IntakeFeedConstants {
  public static final Voltage VOLTAGE = Volts.of(5);
  public static final int CAN_ID = 4;
  public static final double GEAR_RATIO = 3;

  public static class Motor {
    public static final double KP = 1.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.2;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double KA = 0;

    public static final int CURRENT_LIMIT = 60;
  }
}
