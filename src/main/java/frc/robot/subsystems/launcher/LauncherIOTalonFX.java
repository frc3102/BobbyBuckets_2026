package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class LauncherIOTalonFX implements LauncherIO {
  private VelocityTorqueCurrentFOC velocityRequest;
  private TorqueCurrentFOC currentRequest;

  private AngularVelocity referenceVelocity = RotationsPerSecond.of(0);

  // Leader
  private StatusSignal<Current> leaderStatorCurrentStatusSignal;
  private StatusSignal<Current> leaderSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> leaderVelocityStatusSignal;
  private StatusSignal<Temperature> leaderTemperatureStatusSignal;
  private StatusSignal<Voltage> leaderVoltageStatusSignal;

  private LoggedTunableNumber leaderKP =
      new LoggedTunableNumber("Launcher/KP", LauncherConstants.Motor.KP);
  private LoggedTunableNumber leaderKI =
      new LoggedTunableNumber("Launcher/KI", LauncherConstants.Motor.KI);
  private LoggedTunableNumber leaderKD =
      new LoggedTunableNumber("Launcher/KD", LauncherConstants.Motor.KD);
  private LoggedTunableNumber leaderKS =
      new LoggedTunableNumber("Launcher/KS", LauncherConstants.Motor.KS);

  private TalonFX leader;

  private VelocitySystemSim launcherSim;

  // Follower
  private StatusSignal<Current> followerStatorCurrentStatusSignal;
  private StatusSignal<Current> followerSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> followerVelocityStatusSignal;
  private StatusSignal<Temperature> followerTemperatureStatusSignal;
  private StatusSignal<Voltage> followerVoltageStatusSignal;

  private TalonFX follower;

  public LauncherIOTalonFX() {
    leader = new TalonFX(LauncherConstants.CAN_ID_LEADER);
    leaderStatorCurrentStatusSignal = leader.getStatorCurrent();
    leaderSupplyCurrentStatusSignal = leader.getSupplyCurrent();
    leaderVelocityStatusSignal = leader.getVelocity();
    leaderTemperatureStatusSignal = leader.getDeviceTemp();
    leaderVoltageStatusSignal = leader.getMotorVoltage();

    follower = new TalonFX(LauncherConstants.CAN_ID_FOLLOWER);
    followerStatorCurrentStatusSignal = follower.getStatorCurrent();
    followerSupplyCurrentStatusSignal = follower.getSupplyCurrent();
    followerVelocityStatusSignal = follower.getVelocity();
    followerTemperatureStatusSignal = follower.getDeviceTemp();
    followerVoltageStatusSignal = follower.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderStatorCurrentStatusSignal,
        leaderSupplyCurrentStatusSignal,
        leaderVelocityStatusSignal,
        leaderTemperatureStatusSignal,
        leaderVoltageStatusSignal,
        followerStatorCurrentStatusSignal,
        followerSupplyCurrentStatusSignal,
        followerVelocityStatusSignal,
        followerTemperatureStatusSignal,
        followerVoltageStatusSignal);
    if (Constants.TUNING_MODE) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          50.0, leader.getClosedLoopReference(), leader.getClosedLoopError());
    }
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);
    configLeader();
    configFollower();

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));

    this.launcherSim =
        new VelocitySystemSim(leader, LauncherConstants.LEADER_INVERTED, 0.05, 0.01, 1);
  }

  private void configLeader() {
    var config = new TalonFXConfiguration();
    config.Slot0.kS = LauncherConstants.Motor.KS;
    config.Slot0.kP = LauncherConstants.Motor.KP;
    config.Slot0.kI = LauncherConstants.Motor.KI;
    config.Slot0.kD = LauncherConstants.Motor.KD;
    config.CurrentLimits.SupplyCurrentLimit = LauncherConstants.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        LauncherConstants.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
  }

  private void configFollower() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = LauncherConstants.Motor.CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setTargetVelocity(AngularVelocity velocity) {
    leader.setControl(velocityRequest.withVelocity(velocity));
    this.referenceVelocity = velocity.copy();
  }

  @Override
  public void setCurrent(Current amps) {
    leader.setControl(currentRequest.withOutput(amps));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.leaderConnected =
        BaseStatusSignal.isAllGood(
            leaderStatorCurrentStatusSignal,
            leaderSupplyCurrentStatusSignal,
            leaderVelocityStatusSignal,
            leaderTemperatureStatusSignal,
            leaderVoltageStatusSignal);
    inputs.followerConnected =
        BaseStatusSignal.isAllGood(
            followerStatorCurrentStatusSignal,
            followerSupplyCurrentStatusSignal,
            followerVelocityStatusSignal,
            followerTemperatureStatusSignal,
            followerVoltageStatusSignal);

    inputs.leaderAppliedVolts = leaderVoltageStatusSignal.getValue();
    inputs.leaderCurrentAmps = leaderSupplyCurrentStatusSignal.getValue();
    inputs.leaderTemp = leaderTemperatureStatusSignal.getValue();

    inputs.velocity = leaderVelocityStatusSignal.getValue();

    inputs.followerAppliedVolts = followerVoltageStatusSignal.getValue();
    inputs.followerCurrentAmps = followerSupplyCurrentStatusSignal.getValue();
    inputs.followerTemp = followerTemperatureStatusSignal.getValue();

    if (Constants.TUNING_MODE) {
      inputs.errorVelocity = RotationsPerSecond.of(leader.getClosedLoopError().getValueAsDouble());
      inputs.referenceVelocity = this.referenceVelocity.copy();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.leader.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];

          this.leader.getConfigurator().apply(config);
        },
        leaderKP,
        leaderKI,
        leaderKD,
        leaderKS);
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.launcherSim.updateSim();
    }
  }
}
