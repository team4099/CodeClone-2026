
import IntakeRollers.Companion.IntakeRollersState
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

class LinearIntake(private val io: LinearIntakeIO) : ControlledByStateMachine() {
  val inputs = LinearIntakeIO.LintakeIOInputs()
  var currentState = LintakeStates.UNITITALIZED
  var currentRequest: Request.LintakeRequest = Request.LintakeRequest.Idle()
    set(value) {
      when(value){
        is Request.LintakeRequest.Idle -> targetVoltage = IntakeConstants.LinearIntakeConstants.IDLE_VOLTAGE
        is Request.LintakeRequest.OpenLoop -> targetVoltage = value.voltage
        is Request.LintakeRequest.ClosedLoop -> targetPosition = value.position

      }
      field = value
    }
  var targetVoltage = 0.0.volts
  var targetPosition = IntakeConstants.LinearIntakeConstants.START_POSITION
  init{
    if(RobotBase.isReal()){
      LinearIntakeTunableValues.kP.initDefault(
        IntakeConstants.LintakePID.REAL_KP
      )
      LinearIntakeTunableValues.kI.initDefault(
        IntakeConstants.LintakePID.REAL_KI
      )
      LinearIntakeTunableValues.kD.initDefault(
        IntakeConstants.LintakePID.REAL_KD
      )
      LinearIntakeTunableValues.kS.initDefault(
        IntakeConstants.LintakePID.REAL_KS
      )
      LinearIntakeTunableValues.kG.initDefault(
        IntakeConstants.LintakePID.REAL_KG
      )
    }
    else{
      LinearIntakeTunableValues.kP.initDefault(
        IntakeConstants.LintakePID.SIM_KP
      )
      LinearIntakeTunableValues.kI.initDefault(
        IntakeConstants.LintakePID.SIM_KI
      )
      LinearIntakeTunableValues.kD.initDefault(
        IntakeConstants.LintakePID.SIM_KD
      )
      LinearIntakeTunableValues.kS.initDefault(
        IntakeConstants.LintakePID.SIM_KS
      )
      LinearIntakeTunableValues.kG.initDefault(
        IntakeConstants.LintakePID.SIM_KG
      )

    }
    io.configPID(
      LinearIntakeTunableValues.kP.get(),
      LinearIntakeTunableValues.kI.get(),
      LinearIntakeTunableValues.kD.get(),
    )
    io.configFF(
      LinearIntakeTunableValues.kS.get(),
      LinearIntakeTunableValues.kG.get(),
    )
  }

  override fun onLoop() {
    io.updateInputs(inputs)
    CustomLogger.processInputs("Lintake", inputs)
    CustomLogger.recordOutput("Lintake/CurrentState", currentState.name)
    CustomLogger.recordOutput("Lintake/CurrentRequest", currentRequest.javaClass.simpleName)
    CustomLogger.recordOutput("Lintake/TargetVoltage", targetVoltage.inVolts)
    CustomLogger.recordOutput("Lintake/TargetPosition", targetPosition.inInches)

    var nextState = currentState
    when (currentState) {
      LintakeStates.UNITITALIZED -> {
        nextState = LinearIntake.Companion.fromRequestToState(currentRequest)


      }
      LintakeStates.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = LinearIntake.Companion.fromRequestToState(currentRequest)

      }

      LintakeStates.IDLE -> {
        io.setVoltage(IntakeConstants.LinearIntakeConstants.IDLE_VOLTAGE)
        nextState = LinearIntake.Companion.fromRequestToState(currentRequest)
      }

      LintakeStates.CLOSED_LOOP -> {
        io.setPosition(targetPosition)
        nextState = LinearIntake.Companion.fromRequestToState(currentRequest)
      }
    }
    currentState = nextState

  }
  companion object {
    enum class LintakeStates{
      OPEN_LOOP,
      IDLE,
      UNITITALIZED,
      CLOSED_LOOP
    }
    inline fun fromRequestToState(request: Request.LintakeRequest): LintakeStates {
      return when (request) {
        is Request.LintakeRequest.Idle -> LintakeStates.IDLE
        is Request.LintakeRequest.OpenLoop -> LintakeStates.OPEN_LOOP
        is Request.LintakeRequest.ClosedLoop -> LintakeStates.CLOSED_LOOP
      }

    }
  }
}