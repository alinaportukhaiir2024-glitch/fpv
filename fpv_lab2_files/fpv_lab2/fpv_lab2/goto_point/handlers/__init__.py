from fpv_lab2.flight_test.handlers import HANDLERS as BASE_HANDLERS
from fpv_lab2.flight_test.flight_state import FlightState
from fpv_lab2.goto_point.goto_state import GotoState
from fpv_lab2.goto_point.handlers.going_to_target import GoingToTargetHandler
from fpv_lab2.goto_point.handlers.stopping import StoppingHandler

GOTO_HANDLERS = {
    **BASE_HANDLERS,
    FlightState.HOVERING: GoingToTargetHandler(),
    GotoState.GOING_TO_TARGET: GoingToTargetHandler(),
    GotoState.STOPPING: StoppingHandler(),
}
