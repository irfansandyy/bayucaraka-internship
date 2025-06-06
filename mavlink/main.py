from pymavlink import mavutil
import time
import math

def connect() -> mavutil.mavfile:
    connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
    print("Connecting to Plane...")
    connection.wait_heartbeat()
    print("Connected to Plane")
    return connection

def upload_mission(vehicle:mavutil.mavfile):
    time.sleep(0.5)
    print("Clearing mission...")
    vehicle.mav.mission_clear_all_send(
        vehicle.target_system,
        vehicle.target_component
    )
    time.sleep(1)
    print("Mission cleared")
    
    waypoints = [
        [1, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 0.0, 0.0, 0.0, -7.2862805, 112.7890294, 0.0],
        [0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 30.0, 0.0, 0.0, 0.0, -7.2862805, 112.7890294, 40.0],
        [0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 20.0, 0.0, 0.0, -7.2865626, 112.78933530, 55.0],
        [0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 20.0, 0.0, 0.0, -7.2858549, 112.78886320, 55.0],
        [0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 20.0, 0.0, 0.0, -7.2859293, 112.78944250, 55.0],
        [0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 20.0, 0.0, 0.0, -7.2864934, 112.78863790, 55.0],
        [0, mavutil.mavlink.MAV_CMD_DO_JUMP, 2.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]
    
    print("Sending wapoint count...")
    vehicle.mav.mission_count_send(
        vehicle.target_system,
        vehicle.target_component,
        len(waypoints)
    )
    time.sleep(1)
    print("Waypoint count sended")

    print("Initiate waypoint sending")
    for i in range(len(waypoints)):
        time.sleep(0.2)
        wp = waypoints[i]
        print(f"Sending waypoint {i}")
        
        vehicle.mav.mission_item_send(
            vehicle.target_system, 
            vehicle.target_component,
            i,              
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,            
            wp[1],          
            wp[0],  
            1,
            wp[2], wp[3], wp[4], wp[5], 
            wp[6], wp[7], wp[8],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION              
        )
    print("All waypoint are sent")

def start_mission(vehicle:mavutil.mavfile):
    time.sleep(0.5)
    print("Arming plane...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0
    )
    print("Plane armed")
    time.sleep(0.5)
    print("Starting mission...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Mission started")
    
def get_boundary(planeX, planeY):
    startX = -7.2848372
    startY = 112.7881556
    endX = -7.2875883
    endY = 112.7899896
    diffX = endX - startX
    diffY = endY - startY
    
    box = [
        [startX + diffX/2, startY + diffY/2, startX + 3*diffX/4, endY],
        [startX + diffX/4, startY, startX + diffX/2, startY + diffY/2],
        [startX, startY, startX + diffX/4, startY + diffY/2],
        [startX, startY + diffY/2, startX + diffX/4, endY],
        [startX + diffX/4, startY + diffY/2, startX + diffX/2, endY],
        [startX + diffX/2, startY, startX + 3*diffX/4, startY + diffY/2],
        [startX + 3*diffX/4, startY, endX, startY + diffY/2],
        [startX + 3*diffX/4, startY + diffY/2, endX, endY]
    ]
    
    for i, segment in enumerate(box):
        if segment[2] <= planeX <= segment[0] and segment[1] <= planeY <= segment[3]:
            return i+1
    
    return None

def get_lap(vehicle:mavutil.mavfile):
    lap = -1
    time.sleep(10)
    start = time.time_ns()
    seq_before = -1
    while True:
        vehicle.mav.mission_current_send(
            vehicle.target_system,
            vehicle.target_component
        )
        
        msg = vehicle.recv_match(type='MISSION_CURRENT', blocking=True, timeout=2)
        if msg and msg.seq != seq_before:
            seq_before = msg.seq
            if msg.seq == 2:
                lap += 1
                if lap != 0:
                    print(f"Lap {lap} finished")
        
        now = time.time_ns()
        
        if (now - start) % (18 * (10 ** 10)) > 0 and (now - start) % (18 * (10 ** 10)) < (10 ** 10)/2 and (now - start) > (18 * (10 ** 10)):
            vehicle.mav.request_data_stream_send(
                vehicle.target_system, 
                vehicle.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                1, 1
            )
            msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
            
            segment = get_boundary(lat, lon)
            print(f"After {math.floor((now-start)/(6 * (10 ** 10)))} minutes the plane are in segment {segment} and has done {lap} laps")
            time.sleep(5)
            
plane = connect()
upload_mission(plane)
time.sleep(3)
start_mission(plane)
get_lap(plane)