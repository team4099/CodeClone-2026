package com.team4099.robot2026.subsystems.superstructure.shooter.flywheel

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FlywheelConstants
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.inAmpsPerRadianPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadians
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object FlywheelIOTalon : FlywheelIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Flywheel.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Flywheel.FOLLOWER_MOTOR_ID)
  private val motionMagicControl: MotionMagicVelocityTorqueCurrentFOC =
      MotionMagicVelocityTorqueCurrentFOC(-1337.0)
  private val voltReq = VoltageOut(0.0).withEnableFOC(true)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0
  private val slot1Configs: Slot1Configs = configs.Slot1
  private val leaderSensor =
      ctreAngularMechanismSensor(
          leaderTalon, FlywheelConstants.GEAR_RATIO, FlywheelConstants.VOLTAGE_COMPENSATION)
  private val followerSensor =
      ctreAngularMechanismSensor(
          followerTalon, FlywheelConstants.GEAR_RATIO, FlywheelConstants.VOLTAGE_COMPENSATION)

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTorqueCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderVoltageSignal: StatusSignal<WPILibVoltage>
  private var leaderAccelSignal: StatusSignal<WPILibAngularAcceleration>
  private var leaderVelocitySignal: StatusSignal<WPILibAngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTorqueCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
  private var followerVoltageSignal: StatusSignal<WPILibVoltage>
  private var followerAccelSignal: StatusSignal<WPILibAngularAcceleration>
  private var followerVelocitySignal: StatusSignal<WPILibAngularVelocity>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    configs.CurrentLimits.StatorCurrentLimit = FlywheelConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)

    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderTorqueCurrentSignal = leaderTalon.torqueCurrent
    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderVelocitySignal = leaderTalon.velocity
    leaderTempSignal = leaderTalon.deviceTemp
    leaderVoltageSignal = leaderTalon.motorVoltage
    leaderAccelSignal = leaderTalon.acceleration

    followerSupplyCurrentSignal = followerTalon.supplyCurrent
    followerTorqueCurrentSignal = followerTalon.torqueCurrent
    followerStatorCurrentSignal = followerTalon.statorCurrent
    followerVelocitySignal = followerTalon.velocity
    followerTempSignal = followerTalon.deviceTemp
    followerVoltageSignal = followerTalon.motorVoltage
    followerAccelSignal = followerTalon.acceleration

    followerTalon.setControl(
        Follower(Constants.Flywheel.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed))
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        leaderSupplyCurrentSignal,
        leaderStatorCurrentSignal,
        leaderTorqueCurrentSignal,
        leaderVelocitySignal,
        leaderTempSignal,
        leaderVoltageSignal,
        leaderAccelSignal,
        followerStatorCurrentSignal,
        followerTorqueCurrentSignal,
        followerSupplyCurrentSignal,
        followerVelocitySignal,
        followerTempSignal,
        followerVoltageSignal,
        followerAccelSignal,
    )
  }

  override fun updateInputs(inputs: FlywheelIO.FlywheelInputs) {
    updateSignals()
    inputs.flywheelLeaderVelocity = leaderSensor.velocity
    inputs.flywheelLeaderAcceleration =
        (followerAccelSignal.valueAsDouble / FlywheelConstants.GEAR_RATIO)
            .rotations
            .perSecond
            .perSecond
    inputs.flywheelFollowerVelocity = followerSensor.velocity
    inputs.flywheelFollowerAcceleration =
        (followerAccelSignal.valueAsDouble / FlywheelConstants.GEAR_RATIO)
            .rotations
            .perSecond
            .perSecond

    inputs.flywheelLeaderTemperature = leaderTempSignal.valueAsDouble.celsius
    inputs.flywheelLeaderSupplyCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.flywheelLeaderStatorCurrent = leaderStatorCurrentSignal.valueAsDouble.amps
    inputs.flywheelLeaderTorqueCurrent = leaderTorqueCurrentSignal.valueAsDouble.amps
    inputs.flywheelLeaderVoltage = leaderVoltageSignal.valueAsDouble.volts

    inputs.flywheelFollowerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.flywheelFollowerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.flywheelFollowerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.flywheelFollowerStatorCurrent = followerTorqueCurrentSignal.valueAsDouble.amps
    inputs.flywheelFollowerVoltage = followerVoltageSignal.valueAsDouble.volts
  }

  override fun configurePIDCurrent(
      kP0: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI0: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD0: DerivativeGain<Fraction<Radian, Second>, Ampere>,
  ) {
    slot0Configs.kP = kP0.inAmpsPerRadianPerSecond
    slot0Configs.kI = kI0.inAmpsPerRadians
    slot0Configs.kD = kD0.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun configureFFCurrent(
      kS0: StaticFeedforward<Ampere>,
      kV0: VelocityFeedforward<Radian, Ampere>,
      kA0: AccelerationFeedforward<Radian, Ampere>,
  ) {
    slot0Configs.kS = kS0.inAmperes
    slot0Configs.kV = kV0.inAmpsPerRadiansPerSecond
    slot0Configs.kA = kA0.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            lowerBound = -FlywheelConstants.VOLTAGE_COMPENSATION,
            upperBound = FlywheelConstants.VOLTAGE_COMPENSATION)
    leaderTalon.setVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    val slotUsed = 0
    //        if (leaderSensor.velocity < velocity - FlywheelConstants.SHOOTER_TOLERANCE) 1 else 0
    CustomLogger.recordOutput("Flywheel/slotUsed", slotUsed)

    leaderTalon.setControl(
        motionMagicControl
            .withSlot(slotUsed)
            .withVelocity(leaderSensor.velocityToRawUnits(velocity))
            .withAcceleration(
                leaderSensor.accelerationToRawUnits(FlywheelConstants.MAX_ACCELERATION)),
    )
  }
}
