"""Static AABB / robot-sphere features; input geometry is in the same world frame."""
from dataclasses import dataclass
from math import sqrt, isfinite
from .risk import Features

@dataclass(frozen=True)
class GeometrySnapshot:
    stamp_ros_s: float
    map_revision: int
    boxes: tuple  # (xmin,ymin,zmin,xmax,ymax,zmax), metres
    complete: bool = False

class BoxClearance:
    def extract(self, snapshot: GeometrySnapshot, spheres) -> Features:
        # spheres: (x,y,z,radius); must cover chassis, arm and fixed attachments.
        valid = snapshot.complete and bool(spheres)
        for box in snapshot.boxes:
            if len(box) != 6 or not all(map(isfinite, box)) or any(box[i] > box[i+3] for i in range(3)):
                raise ValueError("invalid AABB")
        clearance = float('inf')
        for sphere in spheres:
            if len(sphere) != 4 or not all(map(isfinite, sphere)) or sphere[3] <= 0:
                raise ValueError("invalid robot collision sphere")
            for box in snapshot.boxes:
                distance = sqrt(sum(max(box[i]-sphere[i], 0, sphere[i]-box[i+3])**2 for i in range(3)))
                clearance = min(clearance, distance-sphere[3])
        # Empty complete snapshots still need explicit bounded free-space semantics.
        valid = valid and isfinite(clearance)
        return Features(clearance, 0.0, snapshot.stamp_ros_s, snapshot.map_revision, valid)
