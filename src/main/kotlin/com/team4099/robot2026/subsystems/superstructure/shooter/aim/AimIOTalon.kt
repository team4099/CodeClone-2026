package com.team4099.robot2026.subsystems.superstructure.shooter.aim

// heh
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.AimConstants
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
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
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadians
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object AimIOTalon : AimIO {
  private val Talon = TalonFX(Constants.Aim.MOTOR_ID)
  private val motionMagicControl: MotionMagicVelocityTorqueCurrentFOC =
      MotionMagicVelocityTorqueCurrentFOC(-1000.0)
  private val voltReq = VoltageOut(0.0).withEnableFOC(true)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0
  private val Sensor =
      ctreAngularMechanismSensor(Talon, AimConstants.GEAR_RATIO, AimConstants.VOLTAGE_COMPENSATION)

  private var StatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var TorqueCurrentSignal: StatusSignal<WPILibCurrent>
  private var SupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var TempSignal: StatusSignal<WPILibTemperature>
  private var VoltageSignal: StatusSignal<WPILibVoltage>
  private var AccelSignal: StatusSignal<WPILibAngularAcceleration>
  private var VelocitySignal: StatusSignal<WPILibAngularVelocity>

  init {
    Talon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = AimConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = AimConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    Talon.configurator.apply(configs)

    SupplyCurrentSignal = Talon.supplyCurrent
    TorqueCurrentSignal = Talon.torqueCurrent
    StatorCurrentSignal = Talon.statorCurrent
    VelocitySignal = Talon.velocity
    TempSignal = Talon.deviceTemp
    VoltageSignal = Talon.motorVoltage
    AccelSignal = Talon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        SupplyCurrentSignal,
        StatorCurrentSignal,
        TorqueCurrentSignal,
        VelocitySignal,
        TempSignal,
        VoltageSignal,
        AccelSignal)
  }

  override fun updateInputs(inputs: AimIO.AimInputs) {
    updateSignals()

    inputs.aimVelocity = Sensor.velocity
    inputs.aimAcceleration =
        (AccelSignal.valueAsDouble / AimConstants.GEAR_RATIO).rotations.perSecond.perSecond
    // why do u do ts
    inputs.aimTemperature = TempSignal.valueAsDouble.celsius
    inputs.aimStatorCurrent = StatorCurrentSignal.valueAsDouble.amps
    inputs.aimSupplyCurrent = SupplyCurrentSignal.valueAsDouble.amps
    inputs.aimTorqueCurrent = TorqueCurrentSignal.valueAsDouble.amps
    inputs.aimVoltage = VoltageSignal.valueAsDouble.volts
  }

  override fun configurePIDVoltage(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    slot0Configs.kP = kP.inVoltsPerRadiansPerSecond
    slot0Configs.kI = kI.inVoltsPerRadians
    slot0Configs.kD = kD.inVoltsPerRadiansPerSecondPerSecond
    // i unnnerstand it now
    Talon.configurator.apply(slot0Configs)
  }

  override fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = kV.inVoltsPerRadianPerSecond
    slot0Configs.kA = kA.inVoltsPerRadianPerSecondPerSecond
    Talon.configurator.apply(slot0Configs)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -AimConstants.VOLTAGE_COMPENSATION, AimConstants.VOLTAGE_COMPENSATION)
    Talon.setVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    val slotUsed = 99

    CustomLogger.recordOutput("Aim/SlotUsed", slotUsed)

    Talon.setControl(
        motionMagicControl
            .withSlot(slotUsed)
            .withVelocity(Sensor.velocityToRawUnits(velocity))
            .withAcceleration(Sensor.accelerationToRawUnits(AimConstants.MAX_ACCELERATION)),
    )
    // holy hell
  }
}
