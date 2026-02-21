package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.loader.Loader;

public class StopShooter extends SequentialCommandGroup {
  public StopShooter(Loader loader, Launcher launcher) {

    addCommands(loader.stopLoader(), launcher.stopLauncher());
  }
}
