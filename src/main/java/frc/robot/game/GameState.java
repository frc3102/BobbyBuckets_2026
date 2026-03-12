package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.lib.team6328.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class GameState {

  private GameStateInputsAutoLogged inputs = new GameStateInputsAutoLogged();
  private final GameStateIO io;

  public enum GamePhase {
    PRE_MATCH,
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME,
    POST_MATCH
  }

  public GameState(GameStateIO io) {
    this.io = io;
  }

  public Alliance getAlliance() {
    return inputs.alliance;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GameState", inputs);
    LoggedTracer.record("GameState");
  }

  public boolean isHeadBackWarning() {
    return io.isHeadBackWarning();
  }

  public boolean isGreenLightPreShift() {
    return io.isGreenLightPreShift();
  }

  public boolean isRealMatch() {
    return inputs.matchType != MatchType.None;
  }
}
