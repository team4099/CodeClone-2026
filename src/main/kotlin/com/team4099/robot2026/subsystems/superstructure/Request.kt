package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request {}
  sealed interface RollerRequest : Request {
    class Idle() : RollerRequest
    class OpenLoop(val voltage: ElectricalPotential) : RollerRequest

  }
  sealed interface LintakeRequest : Request {
    class Idle() : LintakeRequest
    class OpenLoop(val voltage: ElectricalPotential) : LintakeRequest
    class ClosedLoop(val position: Length) : LintakeRequest
  }
}
