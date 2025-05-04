#!/usr/bin/env python3
"""
UAV Mission Planner Module.

This module provides the MissionPlanner class to connect to a UAV,
create a square mission, and execute it automatically.
"""

import argparse
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil


class MissionPlanner:
    """Class to plan and execute UAV missions."""

    def __init__(self, connection_string: str):
        """
        Connect to the vehicle.

        Args:
            connection_string: MAVLink connection string (e.g., '127.0.0.1:14550').
        """
        print(f"Connecting to vehicle on {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True)

    def get_location_metres(self, original_location, dNorth: float, dEast: float):
        """
        Return LocationGlobal shifted by dNorth and dEast meters.
        """
        earth_radius = 6378137.0  # meters
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt)

    def get_distance_metres(self, loc1, loc2):
        """
        Compute ground distance in meters between two locations.
        """
        dlat = loc2.lat - loc1.lat
        dlon = loc2.lon - loc1.lon
        return math.sqrt((dlat**2) + (dlon**2)) * 1.113195e5

    def distance_to_current_waypoint(self):
        """
        Return distance to current waypoint or None if at home.
        """
        next_wp = self.vehicle.commands.next
        if next_wp == 0:
            return None
        cmd = self.vehicle.commands[next_wp - 1]
        target = LocationGlobalRelative(cmd.x, cmd.y, cmd.z)
        return self.get_distance_metres(self.vehicle.location.global_frame, target)

    def clear_mission(self):
        """Clear all existing mission commands."""
        cmds = self.vehicle.commands
        cmds.clear()
        cmds.upload()
        print("Cleared existing mission commands.")

    def add_square_mission(self, reference_location, size: float):
        """
        Create a square mission around reference_location with side size*2.
        """
        cmds = self.vehicle.commands
        print("Defining new square mission commands.")
        # Takeoff command
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
        # Four waypoint corners
        points = [
            self.get_location_metres(reference_location,  size, -size),
            self.get_location_metres(reference_location,  size,  size),
            self.get_location_metres(reference_location, -size,  size),
            self.get_location_metres(reference_location, -size, -size),
        ]
        altitudes = [11, 12, 13, 14]
        for pt, alt in zip(points, altitudes):
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                             pt.lat, pt.lon, alt))
        # Dummy waypoint to signal end
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         points[-1].lat, points[-1].lon, 20))
        cmds.upload()
        print("Uploaded square mission commands.")

    def arm_and_takeoff(self, target_altitude: float):
        """Arm vehicle and take off to target_altitude."""
        print("Performing pre-arm checks...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print(f"Taking off to {target_altitude} meters.")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            print(f"Altitude: {altitude:.1f}")
            if altitude >= target_altitude * 0.95:
                print("Reached target altitude.")
                break
            time.sleep(1)

    def execute_mission(self):
        """Start auto mission and monitor waypoints."""
        print("Starting mission.")
        self.vehicle.commands.next = 0
        self.vehicle.mode = VehicleMode("AUTO")
        while True:
            next_wp = self.vehicle.commands.next
            dist = self.distance_to_current_waypoint()
            print(f"Waypoint {next_wp}, distance: {dist}")
            if next_wp == 3:
                print("Skipping to waypoint 5.")
                self.vehicle.commands.next = 5
            if next_wp == 5:
                print("Final waypoint reached. Exiting mission.")
                break
            time.sleep(1)

    def return_to_launch(self):
        """Command vehicle to return to launch and close connection."""
        print("Returning to launch.")
        self.vehicle.mode = VehicleMode("RTL")
        print("Closing vehicle connection.")
        self.vehicle.close()


def main():
    """Parse arguments and execute mission planner."""
    parser = argparse.ArgumentParser(description="UAV Square Mission Planner")
    parser.add_argument(
        "--connection", type=str, default="127.0.0.1:14550",
        help="MAVLink connection string for the vehicle."
    )
    parser.add_argument(
        "--size", type=float, default=50,
        help="Half side length of square mission in meters."
    )
    parser.add_argument(
        "--altitude", type=float, default=10,
        help="Takeoff target altitude in meters."
    )
    args = parser.parse_args()
    planner = MissionPlanner(args.connection)
    planner.clear_mission()
    # Download mission to sync before planning
    planner.vehicle.commands.download()
    planner.vehicle.commands.wait_ready()
    planner.add_square_mission(planner.vehicle.location.global_frame, args.size)
    planner.arm_and_takeoff(args.altitude)
    planner.execute_mission()
    planner.return_to_launch()


if __name__ == "__main__":
    main()
