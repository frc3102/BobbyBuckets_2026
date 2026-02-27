package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public boolean isOurAllianceActive() {
    Alliance active = getCurrentlyActiveAlliance();
    return active == null || inputs.alliance == active;
  }

  public Alliance getCurrentlyActiveAlliance() {
    if (!inputs.receivedGameMessage || inputs.firstActiveAlliance == null) return null;

    Alliance other = (inputs.firstActiveAlliance == Alliance.Blue) ? Alliance.Red : Alliance.Blue;

    switch (inputs.phase) {
      case SHIFT_1:
      case SHIFT_3:
        return inputs.firstActiveAlliance;
      case SHIFT_2:
      case SHIFT_4:
        return other;
      default:
        return null; // Both active (auto, transition, endgame)
    }
  }

  /** Seconds until our next active shift starts. 0 if already active. */
  public double getSecondsUntilOurNextShift() {
    if (!inputs.receivedGameMessage || inputs.firstActiveAlliance == null || isOurAllianceActive())
      return 0;

    double t = DriverStation.getMatchTime();
    boolean weAreFirst = (inputs.alliance == inputs.firstActiveAlliance);

    switch (inputs.phase) {
      case TRANSITION:
        return weAreFirst ? Math.max(0, t - 130) : Math.max(0, t - 105);
      case SHIFT_1:
        return weAreFirst ? 0 : Math.max(0, t - 105);
      case SHIFT_2:
        return weAreFirst ? Math.max(0, t - 80) : 0;
      case SHIFT_3:
        return weAreFirst ? 0 : Math.max(0, t - 55);
      case SHIFT_4:
        return weAreFirst ? Math.max(0, t - 30) : 0;
      default:
        return 0;
    }
  }

  /** True when 5-3 seconds before our shift. Drivers should head to scoring position. */
  public boolean isHeadBackWarning() {
    if (isOurAllianceActive()) return false;
    double s = getSecondsUntilOurNextShift();
    return s > 3.0 && s <= 5.0;
  }

  /** True when 3-0 seconds before our shift. Pre-aim and pre-spool! */
  public boolean isGreenLightPreShift() {
    if (isOurAllianceActive()) return false;
    double s = getSecondsUntilOurNextShift();
    return s > 0.0 && s <= 3.0;
  }

  public Alliance getFirstActiveAlliance() {
    return inputs.firstActiveAlliance;
  }

  public double getTimeRemainingActive() {
    if (!isOurAllianceActive()) return 0;
    double t = DriverStation.getMatchTime();
    boolean weAreFirst = (inputs.alliance == inputs.firstActiveAlliance);
    switch (inputs.phase) {
      case SHIFT_1:
        return weAreFirst ? Math.max(0, t - 105) : Math.max(0, t - 80);
      case SHIFT_2:
        return weAreFirst ? Math.max(0, t - 80) : Math.max(0, t - 55);
      case SHIFT_3:
        return weAreFirst ? Math.max(0, t - 55) : Math.max(0, t - 30);
      case SHIFT_4:
        return weAreFirst ? Math.max(0, t - 30) : Math.max(0, t);
      case END_GAME:
        return Math.max(0, t);
      default:
        return 0;
    }
  }
}
