package frc.robot.subsystems.launcher;

public class LauncherConstants {
  public static final int CAN_ID_LEADER = -1;
  public static final int CAN_ID_FOLLOWER = -1;

  public static final boolean LEADER_INVERTED = false;

  public static class Motor {
    public static final double KP = 1.;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;

    public static final int CURRENT_LIMIT = 60;
  }
}
