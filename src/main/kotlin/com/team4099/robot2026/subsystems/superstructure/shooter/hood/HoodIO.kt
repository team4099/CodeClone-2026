package com.team4099.robot2026.subsystems.superstructure.shooter.hood

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute

interface HoodIO {
  class HoodInputs : LoggableInputs {
    var hoodVelocity = 0.0.volts
    var hoodAcceleration = 0.0.rotations.perMinute.perMinute
    var hoodVoltage = 0.0.volts
    var hoodPosition = 0.0.radians
    var hoodSupplyCurrent = 0.0.amps
    var hoodStatorCurrent = 0.0.amps
    var hoodTorqueCurrent = 0.0.amps
    var hoodTemperature = 0.0.celsius

    override fun fromLog(table: LogTable) {
      table.get("HoodVoltage", hoodVoltage.inVolts).let { hoodVoltage = it.volts }
      table.get("HoodStatorCurrent", hoodStatorCurrent.inAmperes).let {
        hoodStatorCurrent = it.amps
      }
      table.get("HoodSupplyCurrent", hoodSupplyCurrent.inAmperes).let {
        hoodSupplyCurrent = it.amps
      }
      table.get("HoodTorqueCurrent", hoodTorqueCurrent.inAmperes).let {
        hoodTorqueCurrent = it.amps
      }
      table.get("HoodTemperature", hoodTemperature.inCelsius).let { hoodTemperature = it.celsius }
      table.get("HoodVelocity", hoodVelocity.inVolts).let { hoodVelocity = it.volts }
      table.get("HoodAcceleration", hoodAcceleration.inRotationsPerMinutePerMinute).let {
        hoodAcceleration = it.rotations.perMinute.perMinute
      }
      table.get("HoodPosition", hoodPosition.inRadians).let { hoodPosition = it.radians }
    }

    override fun toLog(table: LogTable) {
      table.put("HoodVoltage", hoodVoltage.inVolts)
      table.put("HoodStatorCurrent", hoodStatorCurrent.inAmperes)
      table.put("HoodSupplyCurrent", hoodSupplyCurrent.inAmperes)
      table.put("HoodTorqueCurrent", hoodTorqueCurrent.inAmperes)
      table.put("HoodTemperature", hoodTemperature.inCelsius)
      table.put("HoodVelocity", hoodVelocity.inVolts)
      table.put("HoodAcceleration", hoodAcceleration.inRotationsPerMinutePerMinute)
      table.put("HoodPosition", hoodPosition.inRadians)
    }
  }

  fun setVoltage(voltage: ElectricalPotential) {}

  fun updateInputs(inputs: HoodInputs) {}

  fun configurePIDVoltage(
      kP: ProportionalGain<Radian, Volt>,
      kI: IntegralGain<Radian, Volt>,
      kD: DerivativeGain<Radian, Volt>
  ) {}

  fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>,
  ) {}

  fun setPosition(position: Angle) {}
}
