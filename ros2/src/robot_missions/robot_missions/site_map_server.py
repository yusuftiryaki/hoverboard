"""site_map_server — the site file's keep-outs, obstacles and station as the
latched /obstacle_map Nav2's costmaps read (nav2.yaml static_layer).

    site.geojson ──load──► Site (lat/lon) ──/fromLLArray──► map metres ──► /obstacle_map

Until this node, only robot_sim published /obstacle_map: on the real robot
nothing did, and a StaticLayer with no map never readies its costmap — every
goal aborts, with healthy-looking logs (A3c trap 1). So:

  * An EMPTY or absent site is published IMMEDIATELY as an all-free map: there
    is nothing to convert, and silence would mean "the planner never started",
    not "nothing in the way".
  * A site WITH geometry is published only once navsat_transform's
    /fromLLArray converts it. Before that `map` does not exist either (map->odom
    comes from ekf_global, whose GPS input comes from navsat_transform), so
    there is no window in which Nav2 could plan on an empty map.
  * ⚠️ /fromLLArray does not fail while navsat_transform has no transform — it
    answers an EMPTY list (and /fromLL answers (0, 0, 0) for every point on
    Earth). A short answer is treated as "not ready", asked again a second
    later, and never drawn. docs/missions.md decision 5.
  * The map is built once and latched (transient_local), like robot_sim's.
    navsat_transform computes its transform once (wait_for_datum: false), so
    the conversion cannot go stale under it.

A malformed site file stops this node at startup with the reason. Nav2 then has
no map, plans nothing and the robot stays put — the failure is loud and safe,
where reading half a file would be quiet and dangerous.
"""

from __future__ import annotations

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import FromLLArray

from robot_missions.occupancy import occupied_cells
from robot_missions.site import Site, describe, load_site
from robot_missions.site_map import (
    STATION_BODY_OFFSET, STATION_BODY_RADIUS, MapGeometry, NotReady, assemble,
    build_grid, needs_conversion, positions_to_convert)

DIAG_NAME = "site_map_server: survey map"


class SiteMapServer(Node):
    def __init__(self, **kwargs):
        super().__init__("site_map_server", **kwargs)
        # "" = no site yet: an all-free map, published at once.
        self.declare_parameter("site_file", "")
        self.declare_parameter("resolution", 0.1)
        # Half of global_costmap's 60 m rolling window — see covering_grid.
        self.declare_parameter("margin", 30.0)
        # 3000 x 3000 cells = 300 x 300 m at 0.1 m: ~9 MB per message.
        self.declare_parameter("max_cells", 9_000_000)
        self.declare_parameter("station_body_offset", STATION_BODY_OFFSET)
        self.declare_parameter("station_body_radius", STATION_BODY_RADIUS)
        # docks.yaml: only to notice a station that is NOT on the site (_check_docks).
        self.declare_parameter("dock_database", "")
        self.declare_parameter("retry_period", 1.0)

        p = self.get_parameter
        path = p("site_file").value
        self._site: Site = load_site(path) if path else Site()
        self._status = (DiagnosticStatus.WARN, "waiting for navsat_transform")
        self._problems: list[str] = []
        self._published = False
        self._pending = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._map_pub = self.create_publisher(OccupancyGrid, "obstacle_map", latched)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"site {path or '(none)'}: {describe(self._site)}")
        if not needs_conversion(self._site):
            self._publish(MapGeometry())
            return
        self._from_ll = self.create_client(FromLLArray, "fromLLArray")
        self._asked = []
        self._retry = self.create_timer(p("retry_period").value, self._try_convert)
        self._try_convert()

    # --- conversion ---------------------------------------------------------

    def _try_convert(self) -> None:
        if self._published or self._pending is not None:
            return
        if not self._from_ll.service_is_ready():
            self._status = (DiagnosticStatus.WARN,
                            "waiting for navsat_transform's /fromLLArray service")
            return
        asked = positions_to_convert(self._site)
        request = FromLLArray.Request()
        request.ll_points = [GeoPoint(latitude=q.lat, longitude=q.lon, altitude=0.0)
                             for q in asked]
        self._asked = asked
        self._pending = self._from_ll.call_async(request)
        self._pending.add_done_callback(self._on_answer)

    def _on_answer(self, future) -> None:
        self._pending = None
        response = future.result()
        answer = [(pt.x, pt.y) for pt in response.map_points] if response else []
        try:
            geometry = assemble(self._site, answer, *self._body())
        except NotReady as exc:
            self._status = (DiagnosticStatus.WARN,
                            f"navsat_transform has no transform yet ({exc})")
            self.get_logger().info(
                "navsat_transform not ready yet — /fromLLArray gave "
                f"{len(answer)} of {len(self._asked)} points; asking again",
                throttle_duration_sec=10.0)
            return
        self._publish(geometry)
        self._retry.cancel()

    def _body(self):
        p = self.get_parameter
        return p("station_body_offset").value, p("station_body_radius").value

    # --- output -------------------------------------------------------------

    def _publish(self, geometry: MapGeometry) -> None:
        p = self.get_parameter
        spec, grid = build_grid(geometry, p("resolution").value, p("margin").value,
                                p("max_cells").value)
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = spec.resolution
        message.info.width = spec.width
        message.info.height = spec.height
        message.info.origin.position.x = spec.origin_x
        message.info.origin.position.y = spec.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid
        self._map_pub.publish(message)
        self._published = True

        occupied = occupied_cells(grid)
        self._problems = self._check_docks()
        for problem in self._problems:
            self.get_logger().warn(problem)
        summary = (f"{spec.width} x {spec.height} cells at {spec.resolution} m, "
                   f"{occupied} occupied")
        self.get_logger().info(f"/obstacle_map published: {summary}")
        if self._problems:
            self._status = (DiagnosticStatus.WARN, "; ".join(self._problems))
        else:
            self._status = (DiagnosticStatus.OK, summary)

    def _check_docks(self) -> list[str]:
        """A station in docks.yaml that the site lacks is NOT on this map.

        A site station is the one source of home (battery_supervisor reads it
        too, docs/missions.md 3b), so there is nothing to compare then. But a
        robot running on docks.yaml alone has a station the survey never drew
        — and measured (test_docking_sweep.py), a robot heading past an
        undrawn station drives straight into it."""
        path = self.get_parameter("dock_database").value
        if not path or self._site.stations:
            return []
        with open(path, encoding="utf-8") as handle:
            docks = (yaml.safe_load(handle) or {}).get("docks", {})
        if not docks:
            return []
        return [f"docks.yaml has station(s) {sorted(docks)} but the site has none: the "
                "survey map does not draw it — record the station in the site "
                "(kind: station)"]

    def _publish_diagnostics(self) -> None:
        level, message = self._status
        status = DiagnosticStatus(name=DIAG_NAME, hardware_id="site",
                                  level=level, message=message)
        status.values = [KeyValue(key="site", value=describe(self._site)),
                         KeyValue(key="published", value=str(self._published))]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SiteMapServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
