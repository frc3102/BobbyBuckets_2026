package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.loader.Loader;

public class StartShooter extends SequentialCommandGroup {

  public StartShooter(Loader loader, Launcher launcher) {
    addCommands(launcher.startAtSpeed(RotationsPerSecond.of(2000/60)), new WaitCommand(0.25), loader.startLoader());
  }

}
