interface IntakeRollersIO{
  class RollerInputs : LoggableInputs{
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
      table?.get("rollerAccelerationRotationsPerMinPerMin", rollerAcceleration.inRotationsPerMinutePerMinute){
        rollerAcceleration = it.rotations.perMiniute.perMinute }
      table?.get("rollerVelocity", rollerVelocity.inRotationsPerMinute){
        rollerVelocity = it.rotations.perMinute }
      table?.get("rollerVoltage", rollerAppliedVoltage.inVolts){
        rollerAppliedVoltage = it.volts }

    }
  }
}