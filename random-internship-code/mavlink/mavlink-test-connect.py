from pymavlink import mavutil
import time

def connect_to_simulation():
    """Connect to Mission Planner simulation"""
    print("Attempting to connect to Mission Planner simulation...")
    
    try:
        # Connect to Mission Planner's default simulation port
        connection = mavutil.mavlink_connection(
            'tcp:127.0.0.1:5760',
            source_system=255,
            source_component=0,
            retries=5
        )
        
        # Wait up to 30 seconds for a heartbeat
        print("Waiting for heartbeat...")
        start_time = time.time()
        while time.time() - start_time < 30:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                print("\nHeartbeat received!")
                print(f"Vehicle type: {msg.type}")
                print(f"System status: {msg.system_status}")
                return connection
            print(".", end="", flush=True)
            
        raise TimeoutError("No heartbeat received")
        
    except Exception as e:
        print(f"\nConnection failed: {e}")
        return None

def test_connection(connection):
    """Test if we can get vehicle data"""
    try:
        # Request vehicle parameters
        connection.mav.param_request_list_send(
            connection.target_system,
            connection.target_component
        )
        
        # Try to get some messages
        print("\nTrying to get vehicle data...")
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = connection.recv_match(blocking=True, timeout=1)
            if msg:
                print(f"Message received: {msg.get_type()}")
                
    except Exception as e:
        print(f"Error testing connection: {e}")

def main():
    print("\nMission Planner Simulation Connection Test")
    print("----------------------------------------")
    
    # Step 1: Check if Mission Planner is ready
    print("\nBefore continuing, please ensure:")
    print("1. Mission Planner is open")
    print("2. Simulation is started (Ctrl+Alt+S)")
    print("3. Connected to TCP 127.0.0.1:5760")
    input("\nPress Enter when ready...")
    
    # Step 2: Try to connect
    connection = connect_to_simulation()
    
    if connection:
        print("\nConnection successful!")
        test_connection(connection)
        
        # Monitor connection
        print("\nMonitoring connection (Press Ctrl+C to exit)...")
        try:
            while True:
                msg = connection.recv_match(blocking=True, timeout=1)
                if msg:
                    mtype = msg.get_type()
                    if mtype in ['ATTITUDE', 'GLOBAL_POSITION_INT']:
                        print(f"Received {mtype} message")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nExiting...")
    else:
        print("\nFailed to connect to simulation.")
        print("\nTroubleshooting steps:")
        print("1. Make sure Mission Planner simulation is running")
        print("2. Check that you're connected in Mission Planner")
        print("3. Verify no other program is using port 5760")
        print("4. Try restarting Mission Planner")

if __name__ == "__main__":
    main()