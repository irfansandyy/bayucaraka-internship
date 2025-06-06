from pymavlink import mavutil
import time
from math import radians

def connect_vehicle():
    """Connect to the vehicle and ensure we have a heartbeat"""
    # Connect to SITL simulation
    connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
    
    # Wait for the first heartbeat 
    connection.wait_heartbeat()
    print("Vehicle connected!")
    return connection

def set_home(vehicle, lat, lon, alt):
    """Set the home position of the vehicle"""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, # confirmation
        0, # current position if 1
        0, 0, 0, # unused parameters
        lat, lon, alt
    )
    print(f"Home position set to: Lat: {lat}, Lon: {lon}, Alt: {alt}")

def create_figure8_mission(home_lat, home_lon, size=100):
    """Create waypoints for a figure-8 pattern"""
    waypoints = []
    
    # Convert size from meters to degrees (approximate)
    size_deg = size / 111111.0
    
    # Create figure-8 waypoints
    waypoints.append((home_lat + size_deg, home_lon)) # Top of first circle
    waypoints.append((home_lat, home_lon + size_deg)) # Right of first circle
    waypoints.append((home_lat - size_deg, home_lon)) # Bottom of first circle
    waypoints.append((home_lat, home_lon - size_deg)) # Left of first circle
    waypoints.append((home_lat + size_deg, home_lon)) # Back to top
    waypoints.append((home_lat, home_lon + size_deg)) # Cross to second circle
    waypoints.append((home_lat - size_deg, home_lon)) # Bottom of second circle
    waypoints.append((home_lat, home_lon - size_deg)) # Complete the 8
    waypoints.append((home_lat, home_lon)) # Return home
    
    return waypoints

def upload_mission(vehicle, waypoints, altitude=55):
    """Upload the mission to the vehicle"""
    # Clear current mission
    vehicle.mav.mission_clear_all_send(
        vehicle.target_system,
        vehicle.target_component
    )
    time.sleep(1)
    
    # Send waypoint count
    vehicle.mav.mission_count_send(
        vehicle.target_system,
        vehicle.target_component,
        len(waypoints)
    )
    
    # Send each waypoint
    for i, (lat, lon) in enumerate(waypoints):
        vehicle.mav.mission_item_send(
            vehicle.target_system,
            vehicle.target_component,
            i,  # sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # current waypoint
            1,  # autocontinue
            0, 0, 0, 0,  # params 1-4
            lat, lon, altitude
        )
        time.sleep(0.1)
    
    print(f"Uploaded {len(waypoints)} waypoints")

def main():
    # Connect to the vehicle
    vehicle = connect_vehicle()
    
    # Set home position (example coordinates)
    home_lat = -7.286297
    home_lon = 112.788903
    home_alt = 0
    
    set_home(vehicle, home_lat, home_lon, home_alt)
    time.sleep(1)
    
    # Create and upload figure-8 mission
    waypoints = create_figure8_mission(home_lat, home_lon, size=100)
    upload_mission(vehicle, waypoints, altitude=50)
    
    # Start mission
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Mission started!")
    
    # Monitor mission progress
    while True:
        msg = vehicle.recv_match(type=['MISSION_CURRENT', 'MISSION_ITEM_REACHED'],
                               blocking=True)
        if msg is not None:
            if msg.get_type() == 'MISSION_CURRENT':
                print(f"Current waypoint: {msg.seq}")
            elif msg.get_type() == 'MISSION_ITEM_REACHED':
                print(f"Reached waypoint: {msg.seq}")
                if msg.seq == len(waypoints) - 1:
                    print("Mission completed!")
                    break

main()