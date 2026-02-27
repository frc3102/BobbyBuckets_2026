package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.game.GameState.GamePhase;
import org.littletonrobotics.junction.AutoLog;

public interface GameStateIO {
  @AutoLog
  public static class GameStateInputs {
    public GamePhase phase;
    public Alliance alliance;
    public Alliance firstActiveAlliance;
    public boolean receivedGameMessage;
  }

  public default void updateInputs(GameStateInputs inputs) {}
}
