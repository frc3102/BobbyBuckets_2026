package frc.robot.game;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.game.GameState.GamePhase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class GameStateIORobot implements GameStateIO {
  private Alliance alliance = Alliance.Blue;
  private Alliance firstActiveAlliance;
  private boolean receivedGameMessage = false;
  private GamePhase currentPhase = GamePhase.PRE_MATCH;

  private LoggedDashboardChooser<Alliance> allianceChooser;

  public GameStateIORobot() {
    createChooser();
  }

  @Override
  public void updateInputs(GameStateInputs inputs) {
    updateFirstActiveAlliance();
    updateAlliance();
    updateGamePhase();
    inputs.alliance = alliance;
    inputs.firstActiveAlliance = firstActiveAlliance;
    inputs.phase = currentPhase;
  }

  public void updateFirstActiveAlliance() {
    if (receivedGameMessage) return;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData != null && gameData.length() > 0) {
      char c = gameData.charAt(0);
      if (c == 'B') {
        firstActiveAlliance = Alliance.Blue;
        receivedGameMessage = true;
      } else if (c == 'R') {
        firstActiveAlliance = Alliance.Red;
        receivedGameMessage = true;
      }
    }
  }

  private void updateGamePhase() {
    if (DriverStation.isAutonomous()) {
      currentPhase = GamePhase.AUTO;
    } else if (DriverStation.isTeleop()) {
      double t = DriverStation.getMatchTime();
      if (t > 130) currentPhase = GamePhase.TRANSITION;
      else if (t > 105) currentPhase = GamePhase.SHIFT_1;
      else if (t > 80) currentPhase = GamePhase.SHIFT_2;
      else if (t > 55) currentPhase = GamePhase.SHIFT_3;
      else if (t > 30) currentPhase = GamePhase.SHIFT_4;
      else if (t > 0) currentPhase = GamePhase.END_GAME;
      else currentPhase = GamePhase.POST_MATCH;
    } else {
      // Disabled
      if (currentPhase == GamePhase.END_GAME || currentPhase == GamePhase.POST_MATCH) {
        currentPhase = GamePhase.POST_MATCH;
      } else if (currentPhase != GamePhase.POST_MATCH) {
        currentPhase = GamePhase.PRE_MATCH;
      }
    }
  }

  /**
   * Update Alliance from the FMS. They are not guaranteed to send this data, but if they send it,
   * it can be relied upon. Use manual choice if we haven't gotten FMS data.
   */
  public void updateAlliance() {
    alliance = DriverStation.getAlliance().orElseGet(() -> allianceChooser.get());
  }

  private void createChooser() {
    var chooser = new SendableChooser<Alliance>();
    chooser.addOption("Blue", Alliance.Blue);
    chooser.addOption("Red", Alliance.Red);
    allianceChooser = new LoggedDashboardChooser<>("GameState/ManualAlliance", chooser);
  }
}
