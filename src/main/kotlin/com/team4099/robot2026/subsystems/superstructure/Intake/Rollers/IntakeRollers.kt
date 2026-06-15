import IntakeConstants.RollerIntakeConstants.IDLE_VOLTAGE

import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.ControlledByStateMachine
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Subsystem
import org.team4099.lib.units.derived.volts

class IntakeRollers(private val io: IntakeRollersIO) : ControlledByStateMachine() {
  private var targetVoltage = 0.0.volts
  val inputs = IntakeRollersIO.RollerInputs()
  var currentState = IntakeRollersState.UNINITIALIZED
  var currentRequest: Request.RollerRequest = Request.RollerRequest.Idle()
    set(value) {
      when(value) {
        is Request.IntakeRollerRequest.OpenLoop -> targetVoltage = value.voltage
        is Request.IntakeRollerRequest.Idle -> targetVoltage = IDLE_VOLTAGE
      }
      field = value
    }
  override fun onLoop(){
    io.updateInputs(inputs)
    CustomLogger.processInputs("Intake Rollers", inputs)
    var nextState = currentState
    when (currentState) {
      IntakeRollersState.UNINITIALIZED -> {
        nextState = fromRequestToState(currentRequest)


      }
      IntakeRollersState.OPEN_LOOP -> {
        io.setVoltage(targetVoltage)
        nextState = fromRequestToState(currentRequest)

      }

      IntakeRollersState.IDLE -> {
        io.setVoltage(IDLE_VOLTAGE)
        nextState = fromRequestToState(currentRequest)
      }
    }
    currentState = nextState
  }
  companion object {
    enum class IntakeRollersState{
      OPEN_LOOP,
      IDLE,
      UNINITIALIZED
    }
    inline fun fromRequestToState(request: Request.IntakeRollerRequest): IntakeRollersState {
      return when (request) {
        is Request.IntakeRollerRequest.Idle -> IntakeRollersState.IDLE
        is Request.IntakeRollerRequest.OpenLoop -> IntakeRollersState.OPEN_LOOP

      }
    }
  }



}