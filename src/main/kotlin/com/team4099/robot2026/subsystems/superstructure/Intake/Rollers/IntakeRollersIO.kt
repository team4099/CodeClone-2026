import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute



interface IntakeRollersIO{
  class RollerInputs : LoggableInputs {
    var rollerVelocity = 0.rotations.perMinute
    var rollerAcceleration = 0.rotations.perMinute.perMinute
    var rollerAppliedVoltage = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTemperature = 0.celsius
    override fun toLog(table: LogTable?){
      table?.put("rollerTempCelsius", rollerTemperature.inCelsius)
      table?.put("rollerStratorCurrent", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes)
      table?.put("rollerAccelerationRotationsPerMinPerMin", rollerAcceleration.inRotationsPerMinutePerMinute)
      table?.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
      table?.put("rollerVelocity", rollerVelocity.inRotationsPerMinute)
    }
    override fun fromLog(table: LogTable?){
      table?.get("rollerTempCelsius", rollerTemperature.inCelsius)?.let{
        rollerTemperature = it.celsius}
      table?.get("rollerStratorCurrent", rollerStatorCurrent.inAmperes)?.let{
        rollerStatorCurrent = it.amps}
      table?.get("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes)?.let{
        rollerSupplyCurrent = it.amps}

      table?.get("rollerAccelerationRotationsPerMinPerMin", rollerAcceleration.inRotationsPerMinutePerMinute)?.let{
        rollerAcceleration = it.rotations.perMinute.perMinute }

      table?.get("rollerVelocity", rollerVelocity.inRotationsPerMinute)?.let{
        rollerVelocity = it.rotations.perMinute }

      table?.get("rollerVoltage", rollerAppliedVoltage.inVolts)?.let{
        rollerAppliedVoltage = it.volts }

    }


  }
  fun updateInputs(inputs: RollerInputs) {}
  fun setVoltage(voltage: ElectricalPotential) {}
}