"""Build a local-coordinate square flight path inside a mapped region."""

import math

from shapely.geometry import Point, Polygon


def square_waypoints(region, no_fly_zones, home_lat, home_lon, margin=0.8):
    """Return four corners, the first corner again, and the region center.

    Map coordinates are [longitude, latitude, altitude]. The returned x/y
    coordinates use the same local frame as path_planner and MoveTo.
    """
    if not 0 < margin < 1:
        raise ValueError("square margin must be between 0 and 1")

    def local(point):
        return (
            (point[0] - home_lon) * 111320.0 * math.cos(math.radians(home_lat)),
            (point[1] - home_lat) * 111320.0,
        )

    center = local(region["center"])
    footprint = Polygon([local(point) for point in region["geo_points"]])
    center_point = Point(center)
    if not footprint.is_valid or not footprint.contains(center_point):
        raise ValueError(f"region {region['name']} has no valid center inside its boundary")

    clearance = center_point.distance(footprint.boundary)
    for zone in no_fly_zones:
        obstacle = Polygon([local(point) for point in zone["geo_points"]])
        if not obstacle.is_valid or obstacle.contains(center_point):
            raise ValueError(f"region {region['name']} center intersects a no-fly zone")
        clearance = min(clearance, center_point.distance(obstacle))

    half_side = margin * clearance / math.sqrt(2.0)
    if half_side < 1.0:
        raise ValueError(f"region {region['name']} has insufficient room for a square")

    x, y = center
    corners = [
        (x - half_side, y - half_side),
        (x + half_side, y - half_side),
        (x + half_side, y + half_side),
        (x - half_side, y + half_side),
    ]
    return corners + [corners[0], center]
