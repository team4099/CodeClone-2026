package com.team4099.robot2026.subsystems.superstructure.indexer

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute

interface IndexerIO {
  class IndexerInputs : LoggableInputs {
    var floorTopIndexerVelocity = 0.0.rotations.perMinute
    var floorBottomIndexerVelocity = 0.0.rotations.perMinute
    var floorTopIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var floorBottomIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var floorIndexerTemperature = 0.0.celsius
    var floorIndexerAppliedVoltage = 0.0.volts
    var floorIndexerSupplyCurrent = 0.0.amps
    var floorIndexerStatorCurrent = 0.0.amps

    var sideRollerIndexerVelocity = 0.0.rotations.perMinute
    var sideRollerIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var sideRollerIndexerTemperature = 0.0.celsius
    var sideRollerIndexerAppliedVoltage = 0.0.volts
    var sideRollerIndexerSupplyCurrent = 0.0.amps
    var sideRollerIndexerStatorCurrent = 0.0.amps

    var topBeltIndexerVelocity = 0.0.rotations.perMinute
    var topBeltIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var topBeltIndexerTemperature = 0.0.celsius
    var topBeltIndexerAppliedVoltage = 0.0.volts
    var topBeltIndexerSupplyCurrent = 0.0.amps
    var topBeltIndexerStatorCurrent = 0.0.amps

    var bottomBeltIndexerVelocity = 0.0.rotations.perMinute
    var bottomBeltIndexerAcceleration = 0.0.rotations.perMinute.perMinute
    var bottomBeltIndexerTemperature = 0.0.celsius
    var bottomBeltIndexerAppliedVoltage = 0.0.volts
    var bottomBeltIndexerSupplyCurrent = 0.0.amps
    var bottomBeltIndexerStatorCurrent = 0.0.amps

    override fun toLog(table: LogTable) {
      table.put("floorTopIndexerVelocityRPM", floorTopIndexerVelocity.inRotationsPerMinute)
      table.put("floorBottomIndexerVelocityRPM", floorBottomIndexerVelocity.inRotationsPerMinute)
      table.put(
          "floorTopIndexerAccelerationRPMPM",
          floorTopIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put(
          "floorBottomIndexerAccelerationRPMPM",
          floorBottomIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("floorIndexerSupplyCurrentAmps", floorIndexerSupplyCurrent.inAmperes)
      table.put("floorIndexerStatorCurrentAmps", floorIndexerStatorCurrent.inAmperes)
      table.put("floorIndexerAppliedVoltageVolts", floorIndexerAppliedVoltage.inVolts)
      table.put("floorIndexerTemperatureCelsius", floorIndexerTemperature.inCelsius)

      table.put("sideRollerIndexerVelocityRPM", sideRollerIndexerVelocity.inRotationsPerMinute)
      table.put(
          "sideRollerIndexerAccelerationRPMPM",
          sideRollerIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("sideRollerIndexerSupplyCurrentAmps", sideRollerIndexerSupplyCurrent.inAmperes)
      table.put("sideRollerIndexerStatorCurrentAmps", sideRollerIndexerStatorCurrent.inAmperes)
      table.put("sideRollerIndexerAppliedVoltageVolts", sideRollerIndexerAppliedVoltage.inVolts)
      table.put("sideRollerIndexerTemperatureCelsius", sideRollerIndexerTemperature.inCelsius)

      table.put("topBeltIndexerVelocityRPM", topBeltIndexerVelocity.inRotationsPerMinute)
      table.put(
          "topBeltIndexerAccelerationRPMPM",
          topBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("topBeltIndexerSupplyCurrentAmps", topBeltIndexerSupplyCurrent.inAmperes)
      table.put("topBeltIndexerStatorCurrentAmps", topBeltIndexerStatorCurrent.inAmperes)
      table.put("topBeltIndexerAppliedVoltageVolts", topBeltIndexerAppliedVoltage.inVolts)
      table.put("topBeltIndexerTemperatureCelsius", topBeltIndexerTemperature.inCelsius)

      table.put("bottomBeltIndexerVelocityRPM", bottomBeltIndexerVelocity.inRotationsPerMinute)
      table.put(
          "bottomBeltIndexerAccelerationRPMPM",
          bottomBeltIndexerAcceleration.inRotationsPerMinutePerMinute)
      table.put("bottomBeltIndexerSupplyCurrentAmps", bottomBeltIndexerSupplyCurrent.inAmperes)
      table.put("bottomBeltIndexerStatorCurrentAmps", bottomBeltIndexerStatorCurrent.inAmperes)
      table.put("bottomBeltIndexerAppliedVoltageVolts", bottomBeltIndexerAppliedVoltage.inVolts)
      table.put("bottomBeltIndexerTemperatureCelsius", bottomBeltIndexerTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table.get("floorTopIndexerVelocityRPM", floorTopIndexerVelocity.inRotationsPerMinute).let {
        floorTopIndexerVelocity = it.rotations.perMinute
      }
      table
          .get("floorBottomIndexerVelocityRPM", floorBottomIndexerVelocity.inRotationsPerMinute)
          .let { floorBottomIndexerVelocity = it.rotations.perMinute }
      table
          .get(
              "floorTopIndexerAccelerationRPMPM",
              floorTopIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { floorTopIndexerAcceleration = it.rotations.perMinute.perMinute }
      table
          .get(
              "floorBottomIndexerAccelerationRPMPM",
              floorBottomIndexerAcceleration.inRotationsPerMinutePerMinute)
          .let { floorBottomIndexerAcceleration = it.rotations.perMinute.perMinute }
      table.get("floorIndexerSupplyCurrentAmps", floorIndexerSupplyCurrent.inAmperes).let {
        floorIndexerSupplyCurrent = it.amps
      }
      table.get("floorIndexerStatorCurrentAmps", floorIndexerStatorCurrent.inAmperes).let {
        floorIndexerStatorCurrent = it.amps
      }
      table.get("floorIndexerAppliedVoltageVolts", floorIndexerAppliedVoltage.inVolts).let {
        floorIndexerAppliedVoltage = it.volts
      }
      table.get("floorIndexerTemperatureCelsius", floorIndexerTemperature.inCelsius).let {
        floorIndexerTemperature = it.celsius
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
          .get("bottomBeltIndexerAppliedVoltageVolts", bottomBeltIndexerAppliedVoltage.inVolts)
          .let { bottomBeltIndexerAppliedVoltage = it.volts }
      table.get("bottomBeltIndexerTemperatureCelsius", bottomBeltIndexerTemperature.inCelsius).let {
        bottomBeltIndexerTemperature = it.celsius
      }
    }
  }

  fun setVoltage(voltage: ElectricalPotential) {}

  fun updateInputs(inputs: IndexerInputs) {}

  fun setBrakeMode(brake: Boolean) {}
}
