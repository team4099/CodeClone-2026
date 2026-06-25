package com.team4099.robot2026.subsystems.superstructure.shooter.flywheel

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Second
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
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute

interface FlywheelIO {
  class FlywheelInputs : LoggableInputs {
    var flywheelLeaderVoltage = 0.0.volts
    var flywheelLeaderSupplyCurrent = 0.0.amps
    var flywheelLeaderStatorCurrent = 0.0.amps
    var flywheelLeaderTorqueCurrent = 0.0.amps
    var flywheelLeaderTemperature = 0.0.celsius
    var flywheelLeaderVelocity = 0.0.rotations.perMinute
    var flywheelLeaderAcceleration = 0.0.rotations.perMinute.perMinute

    var flywheelFollowerVoltage = 0.0.volts
    var flywheelFollowerSupplyCurrent = 0.0.amps
    var flywheelFollowerStatorCurrent = 0.0.amps
    var flywheelFollowerTorqueCurrent = 0.0.amps
    var flywheelFollowerTemperature = 0.0.celsius
    var flywheelFollowerVelocity = 0.0.rotations.perMinute
    var flywheelFollowerAcceleration = 0.0.rotations.perMinute.perMinute

    override fun toLog(table: LogTable) {
      table.put("FlywheelLeaderVoltage", flywheelLeaderVoltage.inVolts)
      table.put("FlywheelLeaderSupplyCurrent", flywheelLeaderSupplyCurrent.inAmperes)
      table.put("FlywheelLeaderStatorCurrent", flywheelLeaderStatorCurrent.inAmperes)
      table.put("FlywheelLeaderTorqueCurrent", flywheelLeaderTorqueCurrent.inAmperes)
      table.put("FlywheelLeaderTemperature", flywheelLeaderTemperature.inCelsius)
      table.put("FlywheelLeaderVelocityRPM", flywheelLeaderVelocity.inRotationsPerMinute)
      table.put(
          "FlywheelLeaderAccelerationRPMPM",
          flywheelLeaderAcceleration.inRotationsPerMinutePerMinute)
      table.put("FlywheelFollowerVoltage", flywheelFollowerVoltage.inVolts)
      table.put("FlywheelFollowerSupplyCurrent", flywheelFollowerSupplyCurrent.inAmperes)
      table.put("FlywheelFollowerStatorCurrent", flywheelFollowerStatorCurrent.inAmperes)
      table.put("FlywheelFollowerTorqueCurrent", flywheelFollowerTorqueCurrent.inAmperes)
      table.put("FlywheelFollowerTemperature", flywheelFollowerTemperature.inCelsius)
      table.put("FlywheelFollowerVelocityRPM", flywheelFollowerVelocity.inRotationsPerMinute)
      table.put(
          "FlywheelFollowerAccelerationRPMPM",
          flywheelFollowerAcceleration.inRotationsPerMinutePerMinute)
    }

    override fun fromLog(table: LogTable) {
      // leader
      table.get("FlywheelLeaderVoltage", flywheelLeaderVoltage.inVolts).let {
        flywheelLeaderVoltage = it.volts
      }
      table.get("FlywheelLeaderSupplyCurrent", flywheelLeaderSupplyCurrent.inAmperes).let {
        flywheelLeaderSupplyCurrent = it.amps
      }
      table.get("FlywheelLeaderStatorCurrent", flywheelLeaderStatorCurrent.inAmperes).let {
        flywheelLeaderStatorCurrent = it.amps
      }
      table.get("FlywheelLeaderTorqueCurrent", flywheelLeaderTorqueCurrent.inAmperes).let {
        flywheelLeaderTorqueCurrent = it.amps
      }
      table.get("FlywheelLeaderTemperature", flywheelLeaderTemperature.inCelsius).let {
        flywheelLeaderTemperature = it.celsius
      }
      table.get("FlywheelLeaderVelocityRPM", flywheelLeaderVelocity.inRotationsPerMinute).let {
        flywheelLeaderVelocity = it.rotations.perMinute
      }
      table
          .get(
              "FlywheelLeaderAccelerationRPMPM",
              flywheelLeaderAcceleration.inRotationsPerMinutePerMinute)
          .let { flywheelLeaderAcceleration = it.rotations.perMinute.perMinute }
      // follower
      table.get("FlywheelFollowerVoltage", flywheelFollowerVoltage.inVolts).let {
        flywheelFollowerVoltage = it.volts
      }
      table.get("FlywheelFollowerSupplyCurrent", flywheelFollowerSupplyCurrent.inAmperes).let {
        flywheelFollowerSupplyCurrent = it.amps
      }
      table.get("FlywheelFollowerStatorCurrent", flywheelFollowerStatorCurrent.inAmperes).let {
        flywheelFollowerStatorCurrent = it.amps
      }
      table.get("FlywheelFollowerTorqueCurrent", flywheelFollowerTorqueCurrent.inAmperes).let {
        flywheelFollowerTorqueCurrent = it.amps
      }
      table.get("FlywheelFollowerTemperature", flywheelFollowerTemperature.inCelsius).let {
        flywheelFollowerTemperature = it.celsius
      }
      table.get("FlywheelFollowerVelocityRPM", flywheelFollowerVelocity.inRotationsPerMinute).let {
        flywheelFollowerVelocity = it.rotations.perMinute
      }
      table
          .get(
              "FlywheelFollowerAccelerationRPMPM",
              flywheelFollowerAcceleration.inRotationsPerMinutePerMinute)
          .let { flywheelFollowerAcceleration = it.rotations.perMinute.perMinute }
    }
  }

  fun updateInputs(inputs: FlywheelInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

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

  fun configurePIDCurrent(
      kP0: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI0: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD0: DerivativeGain<Fraction<Radian, Second>, Ampere>
  ) {}

  fun configureFFCurrent(
      kS0: StaticFeedforward<Ampere>,
      kV0: VelocityFeedforward<Radian, Ampere>,
      kA0: AccelerationFeedforward<Radian, Ampere>
  ) {}

  fun setVelocity(velocity: AngularVelocity) {}
}
