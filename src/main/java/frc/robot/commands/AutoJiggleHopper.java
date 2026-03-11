package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.tilt.IntakeTilt;

public class AutoJiggleHopper extends SequentialCommandGroup {

  public AutoJiggleHopper(
      IntakeTilt tilt, double outPosition, double inPosition, double delaySecs) {
    var tiltCommand = new JiggleHopper(tilt, outPosition, inPosition);
    var cmd =
        new ParallelRaceGroup(tiltCommand, new WaitCommand(delaySecs))
            .andThen(new WaitCommand(delaySecs))
            .repeatedly();
    addCommands(cmd);
  }
}
