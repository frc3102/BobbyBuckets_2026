package frc.robot.subsystems.intake.feed;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeFeedTalonFX implements IntakeFeedIO {

  private final TalonFX intake = new TalonFX(IntakeFeedConstants.CAN_ID);
  private final StatusSignal<Angle> intakePositionRot = intake.getPosition();
  private final StatusSignal<AngularVelocity> intakeVelocityRotPerSec = intake.getVelocity();
  private final StatusSignal<Voltage> intakeAppliedVolts = intake.getMotorVoltage();
  private final StatusSignal<Current> intakeCurrentAmps = intake.getSupplyCurrent();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public IntakeFeedTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeFeedConstants.CURRENT_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(5, () -> intake.getConfigurator().apply(intakeConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakePositionRot, intakeVelocityRotPerSec, intakeAppliedVolts, intakeCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(intake);
  }

  @Override
  public void setVoltage(double volts) {
    intake.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void updateInputs(IntakeFeedIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            intakePositionRot, intakeVelocityRotPerSec, intakeAppliedVolts, intakeCurrentAmps);
    inputs.connected = status.isOK();
    inputs.positionRad =
        Units.rotationsToRadians(
            intakePositionRot.getValueAsDouble() / IntakeFeedConstants.MOTOR_REDUCTION);

    inputs.velocityRadPerSec =
        Units.rotationsToRadians(intakeVelocityRotPerSec.getValueAsDouble())
            / IntakeFeedConstants.MOTOR_REDUCTION;
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.currentAmps = intakeCurrentAmps.getValueAsDouble();
  }
}
