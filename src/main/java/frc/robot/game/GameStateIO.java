package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.game.GameState.GamePhase;
import org.littletonrobotics.junction.AutoLog;

public interface GameStateIO {
  @AutoLog
  public static class GameStateInputs {
    public GamePhase phase;
    public Alliance alliance;
    public Alliance firstActiveAlliance;
    public boolean shouldHeadBack;
    public boolean shouldStartShooting;
    public MatchType matchType;
  }

  public default void updateInputs(GameStateInputs inputs) {}

  public default boolean isHeadBackWarning() {
    return false;
  }

  public default boolean isGreenLightPreShift() {
    return false;
  }
}
