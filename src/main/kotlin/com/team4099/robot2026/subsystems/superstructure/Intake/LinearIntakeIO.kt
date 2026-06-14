import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.perSecond



interface LinearIntakeIO{

  class LintakeIOInputs : LoggableInputs {
    var lintakeTemperature = 0.0.celsius
    var lintakeStratorCurrent = 0.amps
    var lintakeSupplyCurrent = 0.amps
    var lintakePosition = 0.inches
    var lintakeVelocity = 0.inches.perSecond
    var lintakeAppliedVoltage = 0.volts


    override fun toLog(table: LogTable?){
      table?.put("lintakeTemperatureCelsius", lintakeTemperature.inCelsius)
      table?.put("lintakeStratorCurrentAmps", lintakeStratorCurrent.inAmperes)
      table?.put("lintakeSupplyCurrentAmps", lintakeSupplyCurrent.inAmperes)
      table?.put("lintakePositionInches", lintakePosition.inInches)
      table?.put("lintakeVelocityInchesPerSec", lintakeVelocity.inInchesPerSecond)
      table?.put("lintakeAppliedVolts", lintakeAppliedVoltage.inVolts)

    }
    override fun fromLog(table: LogTable?){
      table?.get("lintakeTemperatureCelsius", lintakeTemperature.inCelsius)?.let {
        lintakeTemperature = it.celsius
      }
      table?.get("lintakeStatorCurrentAmps", lintakeStratorCurrent.inAmperes)?.let {
        lintakeStratorCurrent = it.amps
      }
      table?.get("lintakeSupplyCurrentAmps", lintakeSupplyCurrent.inAmperes)?.let {
        lintakeSupplyCurrent = it.amps
      }
      table?.get("lintakeAppliedVolts", lintakeAppliedVoltage.inVolts)?.let {
        lintakeAppliedVoltage = it.volts
      }
      table?.get("lintakePositionInches", lintakePosition.inInches)?.let{
        lintakePosition = it.inches}
      table?.get("lintakeVelocityInchesPerSec", lintakeVelocity.inInchesPerSecond)?.let{
        lintakeVelocity = it.inches.perSecond}
    }

  }
  fun updateInputs(inputs: LintakeIOInputs) {}
  fun setVoltage(voltage: ElectricalPotential) {}
  fun setPosition(position: Length) {}
  fun configPID(
    kP: ProportionalGain<Meter, Volt>,
    kI: IntegralGain<Meter, Volt>,
    kD: DerivativeGain<Meter, Volt>
  ) {}
  fun configFF(
    kG: ElectricalPotential,
    kS: StaticFeedforward<Volt>,

    ) {}
  fun setBrakeMode(brake: Boolean) {}
  fun zeroEncoder(){}
}