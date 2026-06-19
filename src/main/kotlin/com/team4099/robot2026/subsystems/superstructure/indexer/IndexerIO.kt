package com.team4099.robot2026.subsystems.superstructure.indexer

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

interface IndexerIO {
  class IndexerInputs : LoggableInputs {
    var floorIndexerVelocity = 0.0.rotations.perMinute
    var floorIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var floorIndexerTemperature = 0.0.celsius
    var floorIndexerAppliedVoltage = 0.0.volts
    var floorIndexerSupplyCurrent = 0.0.amps
    var floorIndexerStatorCurrent = 0.0.amps
    var floorIndexerTorqueCurrent = 0.0.amps

    var feedWheelsIndexerVelocity = 0.0.rotations.perMinute
    var feedWheelsIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var feedWheelsIndexerTemperature = 0.0.celsius
    var feedWheelsIndexerAppliedVoltage = 0.0.volts
    var feedWheelsIndexerSupplyCurrent = 0.0.amps
    var feedWheelsIndexerStatorCurrent = 0.0.amps
    var feedWheelsIndexerTorqueCurrent = 0.0.amps

    var sideRollerIndexerVelocity = 0.0.rotations.perMinute
    var sideRollerIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var sideRollerIndexerTemperature = 0.0.celsius
    var sideRollerIndexerAppliedVoltage = 0.0.volts
    var sideRollerIndexerSupplyCurrent = 0.0.amps
    var sideRollerIndexerStatorCurrent = 0.0.amps
    var sideRollerIndexerTorqueCurrent = 0.0.amps

    var topBeltIndexerVelocity = 0.0.rotations.perMinute
    var topBeltIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var topBeltIndexerTemperature = 0.0.celsius
    var topBeltIndexerAppliedVoltage = 0.0.volts
    var topBeltIndexerSupplyCurrent = 0.0.amps
    var topBeltIndexerStatorCurrent = 0.0.amps
    var topBeltIndexerTorqueCurrent = 0.0.amps

    var bottomBeltIndexerVelocity = 0.0.rotations.perMinute
    var bottomBeltIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var bottomBeltIndexerTemperature = 0.0.celsius
    var bottomBeltIndexerAppliedVoltage = 0.0.volts
    var bottomBeltIndexerSupplyCurrent = 0.0.amps
    var bottomBeltIndexerStatorCurrent = 0.0.amps
    var bottomBeltIndexerTorqueCurrent = 0.0.amps

    override fun toLog(table: LogTable) {
      table.put("floorIndexerVelocityRPM", floorIndexerVelocity.inRotationsPerMinute)
      table.put(
          "floorIndexerAccelerationRPMPM", floorIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("floorIndexerSupplyCurrentAmps", floorIndexerSupplyCurrent.inAmperes)
      table.put("floorIndexerStatorCurrentAmps", floorIndexerStatorCurrent.inAmperes)
      table.put("floorIndexerTorqueCurrentAmps", floorIndexerTorqueCurrent.inAmperes)
      table.put("floorIndexerAppliedVoltageVolts", floorIndexerAppliedVoltage.inVolts)
      table.put("floorIndexerTemperatureCelsius", floorIndexerTemperature.inCelsius)

      table.put("feedWheelsIndexerVelocityRPM", feedWheelsIndexerVelocity.inRotationsPerMinute)
      table.put(
          "feedWheelsIndexerAccelerationRPMPM",
          feedWheelsIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("feedWheelsIndexerSupplyCurrentAmps", feedWheelsIndexerSupplyCurrent.inAmperes)
      table.put("feedWheelsIndexerStatorCurrentAmps", feedWheelsIndexerStatorCurrent.inAmperes)
      table.put("feedWheelsIndexerTorqueCurrentAmps", feedWheelsIndexerTorqueCurrent.inAmperes)
      table.put("feedWheelsIndexerAppliedVoltageVolts", feedWheelsIndexerAppliedVoltage.inVolts)
      table.put("feedWheelsIndexerTemperatureCelsius", feedWheelsIndexerTemperature.inCelsius)

      table.put("sideRollerIndexerVelocityRPM", sideRollerIndexerVelocity.inRotationsPerMinute)
      table.put(
          "sideRollerIndexerAccelerationRPMPM",
          sideRollerIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("sideRollerIndexerSupplyCurrentAmps", sideRollerIndexerSupplyCurrent.inAmperes)
      table.put("sideRollerIndexerStatorCurrentAmps", sideRollerIndexerStatorCurrent.inAmperes)
      table.put("sideRollerIndexerTorqueCurrentAmps", sideRollerIndexerTorqueCurrent.inAmperes)
      table.put("sideRollerIndexerAppliedVoltageVolts", sideRollerIndexerAppliedVoltage.inVolts)
      table.put("sideRollerIndexerTemperatureCelsius", sideRollerIndexerTemperature.inCelsius)

      table.put("topBeltIndexerVelocityRPM", topBeltIndexerVelocity.inRotationsPerMinute)
      table.put(
          "topBeltIndexerAccelerationRPMPM",
          topBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("topBeltIndexerSupplyCurrentAmps", topBeltIndexerSupplyCurrent.inAmperes)
      table.put("topBeltIndexerStatorCurrentAmps", topBeltIndexerStatorCurrent.inAmperes)
      table.put("topBeltIndexerTorqueCurrentAmps", topBeltIndexerTorqueCurrent.inAmperes)
      table.put("topBeltIndexerAppliedVoltageVolts", topBeltIndexerAppliedVoltage.inVolts)
      table.put("topBeltIndexerTemperatureCelsius", topBeltIndexerTemperature.inCelsius)

      table.put("bottomBeltIndexerVelocityRPM", bottomBeltIndexerVelocity.inRotationsPerMinute)
      table.put(
          "bottomBeltIndexerAccelerationRPMPM",
          bottomBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("bottomBeltIndexerSupplyCurrentAmps", bottomBeltIndexerSupplyCurrent.inAmperes)
      table.put("bottomBeltIndexerStatorCurrentAmps", bottomBeltIndexerStatorCurrent.inAmperes)
      table.put("bottomBeltIndexerTorqueCurrentAmps", bottomBeltIndexerTorqueCurrent.inAmperes)
      table.put("bottomBeltIndexerAppliedVoltageVolts", bottomBeltIndexerAppliedVoltage.inVolts)
      table.put("bottomBeltIndexerTemperatureCelsius", bottomBeltIndexerTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table.get("floorIndexerVelocityRPM", floorIndexerVelocity.inRotationsPerMinute).let {
        floorIndexerVelocity = it.rotations.perMinute
      }
      table
          .get(
              "floorIndexerAccelerationRPMPM",
              floorIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { floorIndexerAcceleration = it.rotations.perMinute.perMinute }
      table.get("floorIndexerSupplyCurrentAmps", floorIndexerSupplyCurrent.inAmperes).let {
        floorIndexerSupplyCurrent = it.amps
      }
      table.get("floorIndexerStatorCurrentAmps", floorIndexerStatorCurrent.inAmperes).let {
        floorIndexerStatorCurrent = it.amps
      }
      table.get("floorIndexerTorqueCurrentAmps", floorIndexerTorqueCurrent.inAmperes).let {
        floorIndexerTorqueCurrent = it.amps
      }
      table.get("floorIndexerAppliedVoltageVolts", floorIndexerAppliedVoltage.inVolts).let {
        floorIndexerAppliedVoltage = it.volts
      }
      table.get("floorIndexerTemperatureCelsius", floorIndexerTemperature.inCelsius).let {
        floorIndexerTemperature = it.celsius
      }

      table
          .get("feedWheelsIndexerVelocityRPM", feedWheelsIndexerVelocity.inRotationsPerMinute)
          .let { feedWheelsIndexerVelocity = it.rotations.perMinute }
      table
          .get(
              "feedWheelsIndexerAccelerationRPMPM",
              feedWheelsIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { feedWheelsIndexerAcceleration = it.rotations.perMinute.perMinute }
      table
          .get("feedWheelsIndexerSupplyCurrentAmps", feedWheelsIndexerSupplyCurrent.inAmperes)
          .let { feedWheelsIndexerSupplyCurrent = it.amps }
      table
          .get("feedWheelsIndexerStatorCurrentAmps", feedWheelsIndexerStatorCurrent.inAmperes)
          .let { feedWheelsIndexerStatorCurrent = it.amps }
      table
          .get("feedWheelsIndexerTorqueCurrentAmps", feedWheelsIndexerTorqueCurrent.inAmperes)
          .let { feedWheelsIndexerTorqueCurrent = it.amps }
      table
          .get("feedWheelsIndexerAppliedVoltageVolts", feedWheelsIndexerAppliedVoltage.inVolts)
          .let { feedWheelsIndexerAppliedVoltage = it.volts }
      table.get("feedWheelsIndexerTemperatureCelsius", feedWheelsIndexerTemperature.inCelsius).let {
        feedWheelsIndexerTemperature = it.celsius
      }

      table
          .get("sideRollerIndexerVelocityRPM", sideRollerIndexerVelocity.inRotationsPerMinute)
          .let { sideRollerIndexerVelocity = it.rotations.perMinute }
      table
          .get(
              "sideRollerIndexerAccelerationRPMPM",
              sideRollerIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { sideRollerIndexerAcceleration = it.rotations.perMinute.perMinute }
      table
          .get("sideRollerIndexerSupplyCurrentAmps", sideRollerIndexerSupplyCurrent.inAmperes)
          .let { sideRollerIndexerSupplyCurrent = it.amps }
      table
          .get("sideRollerIndexerStatorCurrentAmps", sideRollerIndexerStatorCurrent.inAmperes)
          .let { sideRollerIndexerStatorCurrent = it.amps }
      table
          .get("sideRollerIndexerTorqueCurrentAmps", sideRollerIndexerTorqueCurrent.inAmperes)
          .let { sideRollerIndexerTorqueCurrent = it.amps }
      table
          .get("sideRollerIndexerAppliedVoltageVolts", sideRollerIndexerAppliedVoltage.inVolts)
          .let { sideRollerIndexerAppliedVoltage = it.volts }
      table.get("sideRollerIndexerTemperatureCelsius", sideRollerIndexerTemperature.inCelsius).let {
        sideRollerIndexerTemperature = it.celsius
      }

      table.get("topBeltIndexerVelocityRPM", topBeltIndexerVelocity.inRotationsPerMinute).let {
        topBeltIndexerVelocity = it.rotations.perMinute
      }
      table
          .get(
              "topBeltIndexerAccelerationRPMPM",
              topBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { topBeltIndexerAcceleration = it.rotations.perMinute.perMinute }
      table.get("topBeltIndexerSupplyCurrentAmps", topBeltIndexerSupplyCurrent.inAmperes).let {
        topBeltIndexerSupplyCurrent = it.amps
      }
      table.get("topBeltIndexerStatorCurrentAmps", topBeltIndexerStatorCurrent.inAmperes).let {
        topBeltIndexerStatorCurrent = it.amps
      }
      table.get("topBeltIndexerTorqueCurrentAmps", topBeltIndexerTorqueCurrent.inAmperes).let {
        topBeltIndexerTorqueCurrent = it.amps
      }
      table.get("topBeltIndexerAppliedVoltageVolts", topBeltIndexerAppliedVoltage.inVolts).let {
        topBeltIndexerAppliedVoltage = it.volts
      }
      table.get("topBeltIndexerTemperatureCelsius", topBeltIndexerTemperature.inCelsius).let {
        topBeltIndexerTemperature = it.celsius
      }

      table
          .get("bottomBeltIndexerVelocityRPM", bottomBeltIndexerVelocity.inRotationsPerMinute)
          .let { bottomBeltIndexerVelocity = it.rotations.perMinute }
      table
          .get(
              "bottomBeltIndexerAccelerationRPMPM",
              bottomBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { bottomBeltIndexerAcceleration = it.rotations.perMinute.perMinute }
      table
          .get("bottomBeltIndexerSupplyCurrentAmps", bottomBeltIndexerSupplyCurrent.inAmperes)
          .let { bottomBeltIndexerSupplyCurrent = it.amps }
      table
          .get("bottomBeltIndexerStatorCurrentAmps", bottomBeltIndexerStatorCurrent.inAmperes)
          .let { bottomBeltIndexerStatorCurrent = it.amps }
      table
          .get("bottomBeltIndexerTorqueCurrentAmps", bottomBeltIndexerTorqueCurrent.inAmperes)
          .let { bottomBeltIndexerTorqueCurrent = it.amps }
      table
          .get("bottomBeltIndexerAppliedVoltageVolts", bottomBeltIndexerAppliedVoltage.inVolts)
          .let { bottomBeltIndexerAppliedVoltage = it.volts }
      table.get("bottomBeltIndexerTemperatureCelsius", bottomBeltIndexerTemperature.inCelsius).let {
        bottomBeltIndexerTemperature = it.celsius
      }
    }
  }

  fun updateInputs(inputs: IndexerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setVelocity(velocity: AngularVelocity) {}

  fun configPIDVoltage(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {}

  fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {}

  fun configurePIDCurrent(
      kP: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD: DerivativeGain<Fraction<Radian, Second>, Ampere>
  ) {}

  fun configureFFCurrent(
      kS: StaticFeedforward<Ampere>,
      kV: VelocityFeedforward<Radian, Ampere>,
      kA: AccelerationFeedforward<Radian, Ampere>
  ) {}

  fun setBrakeMode(brake: Boolean) {}
}
