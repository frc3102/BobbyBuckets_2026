package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class SuperstructureConstants {
  public static final Distance SHOOTER_OFFSET_X = Inches.of(-10);
  public static final Distance SHOOTER_OFFSET_Y = Inches.of(3);

  public static class Conveyor {
    public static final Voltage VOLTAGE = Volts.of(6);
    public static final int CAN_ID = 23;
    public static final double GEAR_RATIO = 3;
    public static final AngularVelocity DEFAULT_VELOCITY = RotationsPerSecond.of(7);

    public static class Motor {
      public static final double KP = .11;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KS = 0.25;
      public static final double KG = 0;
      public static final double KV = .120;
      public static final double KA = 0.01;

      public static final double MM_A = 400;
      public static final double MM_JERK = 4000;

      public static final int CURRENT_LIMIT = 40;
    }
  }

  public static class Kicker {
    public static final Voltage VOLTAGE = Volts.of(6);
    public static final int CAN_ID = 13;
    public static final double GEAR_RATIO = 3;
    public static final AngularVelocity DEFAULT_VELOCITY = RotationsPerSecond.of(-20);

    public static class Motor {
      public static final double KP = 0.11;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KS = 0.25;
      public static final double KV = 0.12;
      public static final double KA = 0.01;

      public static final double MM_A = 400;
      public static final double MM_JERK = 4000;

      public static final int CURRENT_LIMIT = 40;
    }
  }

  public static class Shooter {
    public static final int CAN_ID_LEADER = 22;
    public static final int CAN_ID_FOLLOWER = 14;

    public static final boolean LEADER_INVERTED = false;
    public static final double DEFAULT_SHOOT_SPEED = 40;

    public static class Motor {
      public static final double KP = 0.11;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final double KS = 0.25;
      public static final double KV = 0.12;
      public static final double KA = 0.01;

      public static final double MM_A = 400;
      public static final double MM_JERK = 4000;

      public static final int CURRENT_LIMIT = 60;
    }
  }
}
