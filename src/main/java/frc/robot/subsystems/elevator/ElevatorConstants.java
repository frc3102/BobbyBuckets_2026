package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {
  public static final int CAN_ID = 21;
  public static final double GEAR_RATIO = 1;

  public static final double BOTTOM_POSITION = 0;
  public static final double TOP_POSITION = 43;
  public static final double CLIMB_POSITION = 5;

  public static class Mechanism {
    public static final Distance PULLEY_CIRCUMFERENCE = Inches.of(1);
    public static final Mass ELEVATOR_MASS = Kilograms.of(3);
    // Max and min height are measured at the top of the baby carriage
    public static final Distance MAX_HEIGHT = Inches.of(29.75);
    public static final Distance MIN_HEIGHT = Inches.of(6);
    public static final Distance LINEAR_POSITION_TOLERANCE = Inches.of(0.25);
  }

  public enum Positions {
    BOTTOM(-22),
    TOP(0);

    private double position;

    private Positions(double position) {
      this.position = position;
    }

    public Angle getPosition() {
      return Rotations.of(position);
    }
  }

  public static class Motor {
    public static final double KP = 5.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0.56;
    public static final double KG = 0;
    public static final double KV = .12;
    public static final double KA = .02;
    public static final double MM_CV = 275;
    public static final double MM_A = 250;
    public static final double MM_JERK = 500;

    public static final boolean IS_INVERTED = false;

    public static final int CURRENT_LIMIT = 60;

    public static final Voltage ZEROING_VOLTAGE = Volts.of(-2.0);
  }
}
