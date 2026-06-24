package com.team4099.robot2026.subsystems.superstructure.shooter.aim

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.Temperature
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute

interface AimIO {
  class AimInputs: LoggableInputs {
    var aimVelocity = 0.0.rotations.perMinute
    var aimAcceleration = 0.0.rotations.perMinute.perMinute
    var aimAngle = 0.0.radians
    var aimVoltage = 0.0.volts
    var aimAmps = 0.0.amps
    var aimStatorCurrent = 0.0.amps
    var aimSupplyCurrent = 0.0.amps
    var aimTorqueCurrent = 0.0.amps
    var aimTemperature: Temperature = 0.0.celsius

    override fun fromLog(table: LogTable) {
      table.get("AimVoltage", aimVoltage.inVolts).let { aimVoltage = it.volts }
      table.get("AimAmps", aimAmps.inAmperes).let { aimAmps = it.amps }
      table.get("AimStatorCurrent", aimStatorCurrent.inAmperes).let { aimStatorCurrent = it.amps }
      table.get("AimSupplyCurrent", aimSupplyCurrent.inAmperes).let { aimSupplyCurrent = it.amps }
      table.get("AimTorqueCurrent", aimTorqueCurrent.inAmperes).let { aimTorqueCurrent = it.amps }
      table.get("AimTemperature", aimTemperature.inCelsius).let { aimTemperature = it.celsius }
      table.get("AimVelocity", aimVelocity.inRotationsPerMinute).let { aimVelocity = it.rotations.perMinute }
      table.get("AimAcceleration", aimAcceleration.inRotationsPerMinutePerMinute).let { aimAcceleration = it.rotations.perMinute.perMinute }
      table.get("AimAngle", aimAngle.inRadians).let { aimAngle = it.radians }
    }

    override fun toLog(table: LogTable) {
      table.put("AimVoltage", aimVoltage.inVolts)
      table.put("AimAmps", aimAmps.inAmperes)
      table.put("AimStatorCurrent", aimStatorCurrent.inAmperes)
      table.put("AimSupplyCurrent", aimSupplyCurrent.inAmperes)
      table.put("AimTorqueCurrent", aimTorqueCurrent.inAmperes)
      table.put("AimTemperature", aimTemperature.inCelsius)
      table.put("AimVelocity", aimVelocity.inRotationsPerMinute)
      table.put("AimAcceleration", aimAcceleration.inRotationsPerMinutePerMinute)
      table.put("AimAngle", aimAngle.inRadians)
    }
  }

  fun updateInputs(inputs: AimInputs){}

  fun setVelocity(velocity: AngularVelocity){}

  fun setVoltage(voltage: ElectricalPotential){}

  fun configurePIDVoltage(
    kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
    kI: IntegralGain<Fraction<Radian, Second>, Volt>,
    kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {}

  fun configureFFVoltage(
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>,
  ) {}

}
