package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current as WPICurrent
import edu.wpi.first.units.measure.Temperature as WPITemp
import edu.wpi.first.units.measure.Voltage as WPIVolt
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeRollersIOTalon : IntakeRollersIO {
  private val rollerTalon: TalonFX = TalonFX(Constants.Rollers.ROLLERS_MOTOR_ID)

  private val rollerConfig: TalonFXConfiguration = TalonFXConfiguration()
  private var rollerSensor =
      ctreAngularMechanismSensor(
          IntakeRollersIOTalon.rollerTalon,
          IntakeConstants.RollerIntakeConstants.GEAR_RATIO,
          IntakeConstants.RollerIntakeConstants.VOLTAGE_COMPENSATION)

  var statorCurrentSignal: StatusSignal<WPICurrent>
  var supplyCurrentSignal: StatusSignal<WPICurrent>
  var temperatureSignal: StatusSignal<WPITemp>
  var motorVoltage: StatusSignal<WPIVolt>
  var motorAcceleration: StatusSignal<WPIAngularAcceleration>
  var motorVelocity: StatusSignal<WPIAngularVelocity>
  val voltageOut = VoltageOut(-1337.0)

  init {
    rollerTalon.clearStickyFaults()

    rollerConfig.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.RollerIntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.RollerIntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    supplyCurrentSignal = rollerTalon.supplyCurrent
    statorCurrentSignal = rollerTalon.statorCurrent
    temperatureSignal = rollerTalon.deviceTemp
    motorVelocity = rollerTalon.velocity
    motorAcceleration = rollerTalon.acceleration
    motorVoltage = rollerTalon.motorVoltage

    rollerTalon.configurator.apply(rollerConfig)
  }

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    refreshStatusSignal()

    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.rollerStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.rollerSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.rollerTemperature = temperatureSignal.valueAsDouble.celsius
    inputs.rollerAcceleration =
        motorAcceleration.valueAsDouble.rotations.perSecond.perSecond *
            IntakeConstants.RollerIntakeConstants.GEAR_RATIO
  }

  fun refreshStatusSignal() {
    BaseStatusSignal.refreshAll(
        supplyCurrentSignal,
        temperatureSignal,
        statorCurrentSignal,
        motorVoltage,
        motorAcceleration,
        motorVelocity,
    )
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            -IntakeConstants.RollerIntakeConstants.VOLTAGE_COMPENSATION,
            IntakeConstants.RollerIntakeConstants.VOLTAGE_COMPENSATION)
    rollerTalon.setControl(voltageOut.withOutput(clampedVoltage.value))
  }
}
