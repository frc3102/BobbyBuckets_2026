package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import org.littletonrobotics.junction.AutoLog;

public interface HapticsIO {
  @AutoLog
  public static class HapticsIOInputs {
    public RumbleType type;
    public double strength;
  }

  public default void updateInputs(HapticsIOInputs inputs) {}

  public default void setRumble(RumbleType type, double strength) {}

  public default void stop() {}
}
