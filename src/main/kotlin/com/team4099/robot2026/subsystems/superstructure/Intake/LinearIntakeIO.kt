
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
      table?.put("lintakePositionMeters", lintakePosition.inInches)
      table?.put("lintakeVelocityMetersPerMin", lintakeVelocity.inInchesPerMinute)
      table?.put("lintakeAppliedVolts", lintakeAppliedVoltage.inVolts)

    }
  }
}