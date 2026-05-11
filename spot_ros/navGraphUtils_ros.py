"""ROS-compatible stubs for GraphNav utilities.

These keep the EasyWalk flow running in simulation without GraphNav.
"""
from dataclasses import dataclass


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    cell_row: int | None = None
    cell_col: int | None = None


class RecordingInterface:
    def __init__(self, *args, **kwargs):
        self.waypoints = []
        self._download_filepath = None

    def stop_recording(self, *args, **kwargs):
        return True

    def start_recording(self, *args, **kwargs):
        return True

    def clear_map(self, *args, **kwargs):
        self.waypoints.clear()
        return True

    def set_download_filepath(self, filepath):
        self._download_filepath = filepath

    def create_default_waypoint(self, cell_row=None, cell_col=None, x=0.0, y=0.0, z=0.0, yaw=0.0):
        ##### info sul robot sono passate come parametri ####
        name = f"wp_{len(self.waypoints)}"
        wp = Waypoint(name=name, x=x, y=y, z=z, yaw=yaw, cell_row=cell_row, cell_col=cell_col)
        self.waypoints.append(wp)
        return wp

    def get_recording_status(self, *args, **kwargs):
        return True

    def get_all_manual_waypoints_with_cells(self):
        data = {}
        for wp in self.waypoints:
            if wp.cell_row is None or wp.cell_col is None:
                continue
            data[(wp.cell_row, wp.cell_col)] = {
                'id': wp.name,
                'name': wp.name,
                'x': wp.x,
                'y': wp.y,
                'z': wp.z,
                'yaw': wp.yaw,
            }
        return data

    def get_manual_waypoint_by_cell(self, cell_row, cell_col):
        for wp in self.waypoints:
            if wp.cell_row == cell_row and wp.cell_col == cell_col:
                return {'id': wp.name, 'name': wp.name, 'x': wp.x, 'y': wp.y, 'z': wp.z, 'yaw': wp.yaw}
        return None

    def find_nearest_waypoint_cell_to_target(self, target_cell, waypoints_by_cell, env_map):
        if not waypoints_by_cell:
            return None
        target_row, target_col = target_cell
        best_cell = None
        best_dist = 1e9
        for (row, col) in waypoints_by_cell.keys():
            d = abs(row - target_row) + abs(col - target_col)
            if d < best_dist:
                best_dist = d
                best_cell = (row, col)
        return best_cell

    def navigate_to_waypoint(self, *args, **kwargs):
        return True

    def auto_close_loops(self, *args, **kwargs):
        return True

    def optimize_anchoring(self, *args, **kwargs):
        return True

    def navigate_to_first_waypoint(self, *args, **kwargs):
        return True

    def download_full_graph(self, *args, **kwargs):
        return self._download_filepath
