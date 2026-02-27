package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class HapticsIOXboxController implements HapticsIO {
  private final CommandXboxController controller;

  private RumbleType type;
  private double strength;

  public HapticsIOXboxController(CommandXboxController controller) {
    this.controller = controller;
  }

  @Override
  public void updateInputs(HapticsIOInputs inputs) {
    inputs.type = type;
    inputs.strength = strength;
  }

  @Override
  public void stop() {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void setRumble(RumbleType type, double strength) {
    controller.setRumble(type, strength);
  }
}
