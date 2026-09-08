"""The earth's magnetic field as this project simulates it. ROS-free.

⚠️ THIS FILE EXISTS BECAUSE THIS MODEL WAS ONCE IN TWO PLACES. sim_node.py and
fake_bus.py each carried their own copy of R(-yaw) @ (0, N), each with the same
sign flipped, and a test helper inverted it a third time — three wrongs whose
product was the identity. Seventeen tests stayed green against a mirrored earth
while A2's GPS waypoints looked like a broken Nav2 controller, for weeks. That
is A6 in docs/handoff.md.

The fix is not "be more careful next time". It is that there is now exactly one
copy of the model to be wrong, so a mistake in it fails everything at once
instead of cancelling itself. Anything needing the simulated field imports it
from here and never re-derives it.

It lives in the magnetometer's package rather than in robot_sim because the
driver has to be deployable on the robot without the simulator; the dependency
only makes sense pointing this way.
"""

from __future__ import annotations

import math
from typing import Tuple

# Istanbul-ish. The horizontal component is what makes a heading observable;
# the vertical only matters once the robot tilts.
EARTH_NORTH_T = 26e-6      # horizontal, pointing north, tesla
EARTH_DOWN_T = 36e-6       # vertical, pointing DOWN, tesla


def field_in_body_frame(yaw: float) -> Tuple[float, float, float]:
    """The earth's field seen by a level sensor on a robot facing `yaw`.

    REP-103: x forward, y left, z up, and yaw 0 = facing east — so the earth's
    horizontal field, pointing north, lies along +y there. Turning the robot by
    yaw rotates the field by -yaw in the body frame:

        R(-yaw) @ (0, N) = (N sin yaw, N cos yaw)

    ⚠️ Check the signs against a HEADING, never against the algebra: facing
    north (yaw = +90 deg) the field must lie straight along FORWARD, so
    x = +N and y = 0. Checking at yaw = 0 proves nothing at all — sin(0) = 0,
    so a mirrored field and a correct one are byte-identical exactly there,
    which is why A6 survived a full green suite.
    """
    return (
        EARTH_NORTH_T * math.sin(yaw),
        EARTH_NORTH_T * math.cos(yaw),
        -EARTH_DOWN_T,                 # down is -z in REP-103
    )
