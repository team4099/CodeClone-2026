
interface LinearIntakeIO{
  class LintakeIOInputs : LoggableInputs{
    var lintakeTemperature = 0.0.celsius
    var lintakeStratorCurrent = 0.amps
    var lintakeSupplyCurrent = 0.amps
    var lintakeAppliedVoltage = 0.volts
    var lintakeVelocity = 0.0.inches.perSecond
    var lintakePosition = 0.0.inces
    override fun toLog(table: LogTable?){
      table?.put("lintakeTemperatureCelsius", lintakeTemperature.inCelsius)
      table?.put("lintakeStatorCurrentAmps", lintakeStatorCurrent.inAmperes)
      table?.put("lintakeSupplyCurrentAmps", lintakeSupplyCurrent.inAmperes)
      table?.put("lintakePositionInches", lintakePosition.inInches)
      table?.put("lintakeVelocityInchesPerMin", lintakeVelocity.inInchesPerMinute)
      table?.put("lintakeAppliedVolts", lintakeAppliedVoltage.inVolts)

    }
    override fun fromLog(table: LogTable?){
      table?.get("lintakeTemperatureCelsius", lintakeTemperature.inCelsius)?.let {
        lintakeTemperature = it.celsius
      }
      table?.get("lintakeStatorCurrentAmps", lintakeStratorCurrent.iAmperes)?.let {
        lintakeStratorCurrent = it.amps
      }
      table?.get("lintakeSupplyCurrentAmps", lintakeSupplyCurrent.iAmperes)?.let {
        lintakeSupplyCurrent = it.amps
      }
      table?.get("lintakeAppliedVolts", lintakeAppliedVoltage.inVolts)?.let {
        lintakeAppliedVoltage = it.volts
      }
      table?.get("lintakePositionInches", lintakePosition.inInches)?.let{
        lintakePosition = it.inches}
      table?.get("lintakeVelocityInchesPerMin", lintakeVelocity.inInchesPerMinute)?.let{
        lintakeVelocity = it.inches.perMinute}
    }
    fun updateInputs(inputs: LintakeIOInputs) {}
    fun setVoltage(voltage: ElectricalPotential) {}
    fun setPosition(position: Length) {}
    fun configPID(
      kP: ProportionalGain<Inches, Volt>,
      kI: IntegralGain<Inches, Volt>,
      kD: DerivativeGain<Inches, Volt>
    ) {}
    fun configFF(
      kG: ElectricalPotential,
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Inches, Volt>,
      kA: AccelerationFeedforward<Inches, Volt>
    ) {}
    fun setBrakeMode(brake: Boolean) {}
  }
}