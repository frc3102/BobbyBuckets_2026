package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team6328.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Haptics extends SubsystemBase {

  private final HapticsIO io;
  private final HapticsIOInputsAutoLogged inputs = new HapticsIOInputsAutoLogged();

  public Haptics(HapticsIO io) {
    this.io = io;
  }

  private Command stopRumble() {
    return runOnce(() -> io.stop());
  }

  private Command startRumble(RumbleType type, double strength) {
    return runOnce(() -> io.setRumble(type, strength));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Haptics", inputs);
    LoggedTracer.record("Haptics");
  }

  public static class RumblePatternCommand extends SequentialCommandGroup {
    public RumblePatternCommand(
        Haptics haptics,
        int count,
        double buzzDuration,
        double buzzGap,
        RumbleType type,
        double strength) {
      addRequirements(haptics);
      for (int i = 0; i < count; i++) {
        addCommands(
            haptics.startRumble(type, strength),
            new WaitCommand(buzzDuration),
            haptics.stopRumble(),
            new WaitCommand(buzzGap));
      }
    }
  }
}
