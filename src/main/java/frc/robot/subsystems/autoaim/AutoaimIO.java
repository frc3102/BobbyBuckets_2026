package frc.robot.subsystems.autoaim;

import org.littletonrobotics.junction.AutoLog;

public interface AutoaimIO {
  @AutoLog
  public static class AutoaimIOInputs {}

  public default void updateInputs(AutoaimIOInputs inputs) {}
}
