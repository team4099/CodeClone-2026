package com.team4099.robot2026.subsystems.superstructure.feeder

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

interface FeederIO {
  class FeederInputs : LoggableInputs {
    var feederVelocity = 0.0.rotations.perMinute
    var feederAcceleration = 0.0.rotations.perMinute.perMinute
    var feederTemperature = 0.0.celsius
    var feederAppliedVoltage = 0.0.volts
    var feederSupplyCurrent = 0.0.amps
    var feederStatorCurrent = 0.0.amps

    override fun toLog(table: LogTable) {
      table.put("feederVelocityRPM", feederVelocity.inRotationsPerMinute)
      table.put("feederAccelerationRPMPM", feederAcceleration.inRotationsPerMinutePerMinute)
      table.put("feederSupplyCurrentAmps", feederSupplyCurrent.inAmperes)
      table.put("feederStatorCurrentAmps", feederStatorCurrent.inAmperes)
      table.put("feederAppliedVoltageVolts", feederAppliedVoltage.inVolts)
      table.put("feederTemperatureCelsius", feederTemperature.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table.get("feederVelocityRPM", feederVelocity.inRotationsPerMinute).let {
        feederVelocity = it.rotations.perMinute
      }
      table.get("feederAccelerationRPMPM", feederAcceleration.inRotationsPerMinutePerMinute).let {
        feederAcceleration = it.rotations.perMinute.perMinute
      }
      table.get("feederSupplyCurrentAmps", feederSupplyCurrent.inAmperes).let {
        feederSupplyCurrent = it.amps
      }
      table.get("feederStatorCurrentAmps", feederStatorCurrent.inAmperes).let {
        feederStatorCurrent = it.amps
      }
      table.get("feederAppliedVoltageVolts", feederAppliedVoltage.inVolts).let {
        feederAppliedVoltage = it.volts
      }
      table.get("feederTemperatureCelsius", feederTemperature.inCelsius).let {
        feederTemperature = it.celsius
      }
    }
  }

  fun updateInputs(inputs: FeederInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}
}
