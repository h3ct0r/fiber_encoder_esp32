import time
import math
from dynamixel_sdk import *
import csv

# --- Configuration ---
# Match these to your setup
DXL_ID                      = 1                            # Dynamixel ID
BAUDRATE                    = 1000000                      # AX-12+ default baud rate is 1000000 (try 57600 or 115200)
DEVICENAME                  = '/dev/cu.usbserial-A600cUDI' # Use your actual device name from Dynamixel Wizard
PROTOCOL_VERSION            = 1.0                          # AX-12+ uses Protocol 1.0, not 2.0

# --- AX-12+ Control Table Addresses ---
ADDR_AX_TORQUE_ENABLE       = 24
ADDR_AX_CW_ANGLE_LIMIT      = 6
ADDR_AX_CCW_ANGLE_LIMIT     = 8
ADDR_AX_MOVING_SPEED        = 32

portHandler = None
packetHandler = None

def open_dynamixel_connection():
    global portHandler, packetHandler

    if portHandler is not None and packetHandler is not None:
        print("Dynamixel connection is already open. Skipping open.")
        return

    while True:
        try:
            portHandler = PortHandler(DEVICENAME)
            packetHandler = PacketHandler(PROTOCOL_VERSION)

            # Open port
            if not portHandler.openPort():
                print(f"Failed to open the port {DEVICENAME}")
                raise Exception("Port open failed")

            # Set port baudrate
            if not portHandler.setBaudRate(BAUDRATE):
                print(f"Failed to change the baudrate to {BAUDRATE}")
                raise Exception("Baudrate change failed")

            # Set to Wheel Mode (by setting both angle limits to 0)
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_CW_ANGLE_LIMIT, 0)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(f"Failed to set CW angle limit: {packetHandler.getTxRxResult(dxl_comm_result)}")
                print(f"CW angle limit error: {packetHandler.getRxPacketError(dxl_error)}")
                raise Exception("Failed to set CW angle limit")
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_CCW_ANGLE_LIMIT, 0)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(f"Failed to set CCW angle limit: {packetHandler.getTxRxResult(dxl_comm_result)}")
                print(f"CCW angle limit error: {packetHandler.getRxPacketError(dxl_error)}")
                raise Exception("Failed to set CCW angle limit")

            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, 1)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(f"{packetHandler.getTxRxResult(dxl_comm_result)}")
                print(f"{packetHandler.getRxPacketError(dxl_error)}")
                raise Exception("Failed to enable torque")
            
            break  # Successfully opened connection
        except Exception as e:
            try:
                if portHandler is not None:
                    portHandler.closePort()
            except Exception as close_error:
                print(f"Error while closing port: {close_error}")
            portHandler = None
            packetHandler = None
            print(f"Error while opening Dynamixel connection: {e}. Press any key to try again...")
            input()  # Wait for user input to try again
            continue

def set_dynamixel_speed(dynamixel_speed_value):
    global portHandler, packetHandler

    if portHandler is None or packetHandler is None:
        print("Dynamixel connection is not open.")
        exit()

    attempts = 3
    while attempts > 0:
        # Write goal speed
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, dynamixel_speed_value)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"Failed to set moving speed: {packetHandler.getTxRxResult(dxl_comm_result)}")
            print(f"Moving speed error: {packetHandler.getRxPacketError(dxl_error)}")
            attempts -= 1
            if attempts == 0:
                print("Exceeded maximum attempts to set speed. Exiting.")
                exit()
            print("Retrying to set speed...")
            continue
        break  # Successfully set speed

def close_dynamixel_connection():
    global portHandler, packetHandler

    if portHandler is None or packetHandler is None:
        print("Dynamixel connection is not open. Skipping close.")
        return

    # Set speed to 0, disable torque, and close port
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_MOVING_SPEED, 0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, 0)
    portHandler.closePort()

    portHandler = None
    packetHandler = None

def move(wheel_diameter_cm, target_travel_distance_cm, target_linear_speed_cm_s, clockwise, max_motor_rpm):
    # --- Calculated Speed Value ---
    # 1. Calculate wheel circumference
    wheel_circumference_cm = wheel_diameter_cm * math.pi

    # 2. Calculate target revolutions per second (RPS)
    revolutions_per_second = target_linear_speed_cm_s / wheel_circumference_cm

    # 3. Calculate target revolutions per minute (RPM)
    revolutions_per_minute = revolutions_per_second * 60

    # 4. Convert RPM to Dynamixel speed value (0-1023)
    # The range 0-1023 is for Counter-Clockwise (CCW) rotation.
    # To spin Clockwise (CW), add 1024 to the value.

    rpm_per_unit = max_motor_rpm / 1023
    dynamixel_speed_value = int(revolutions_per_minute / rpm_per_unit)

    # Ensure the speed value is within the valid range
    if dynamixel_speed_value > 1023:
        print(f"Warning: Calculated speed {dynamixel_speed_value} exceeds maximum 1023, capping to 1023")
        dynamixel_speed_value = 1023
    if dynamixel_speed_value < 1:
        print(f"Warning: Calculated speed {dynamixel_speed_value} is too low, setting to minimum 1")
        dynamixel_speed_value = 1
        
    if clockwise:
        dynamixel_speed_value = dynamixel_speed_value + 1024

    time_to_travel_s = target_travel_distance_cm / target_linear_speed_cm_s

    print(f"{target_linear_speed_cm_s}cm/s {'CW' if clockwise else 'CCW'}\tRPM: {revolutions_per_minute:.2f}\tDynamixel Speed: {dynamixel_speed_value}\tDistance {target_travel_distance_cm}cm\tETA {time_to_travel_s:.2f}s")
    print(f"Press Ctrl+C to stop.")

    open_dynamixel_connection()
    try:
        set_dynamixel_speed(dynamixel_speed_value)
        # Let it run for the calculated time
        time.sleep(time_to_travel_s)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Stopping motor...")
    finally:
        set_dynamixel_speed(0)
        close_dynamixel_connection()

# =========== 4CM WHEEL SETTINGS ===========

experiments = []
def read_experiments_from_csv():
    global experiments
    with open('experiments.csv', newline='') as csvfile:
        experiments = []
        reader = csv.DictReader(csvfile)
        for row in reader:
            experiments.append({
                "wheel_diameter_cm": float(row["wheel_diameter_cm"]),
                "target_travel_distance_cm": float(row["target_travel_distance_cm"]),
                "forward_linear_speed_cm_s": float(row["forward_linear_speed_cm_s"]),
                "forward_max_motor_rpm": float(row["forward_max_motor_rpm"]),
                "backward_linear_speed_cm_s": float(row["backward_linear_speed_cm_s"]),
                "backward_max_motor_rpm": float(row["backward_max_motor_rpm"])
            })

# Read initial experiments from CSV
read_experiments_from_csv()

while True:
    i = -1
    while i == -1:
        read_experiments_from_csv()
        print("=" * 50)
        print(f"Available Experiments:")
        for idx, exp in enumerate(experiments):
            print(f"({idx + 1})\tWheel: {exp['wheel_diameter_cm']}cm\tForward: {exp['forward_linear_speed_cm_s']}cm/s\tMax RPM: {exp['forward_max_motor_rpm']}\tBackward: {exp['backward_linear_speed_cm_s']}cm/s\tMax RPM: {exp['backward_max_motor_rpm']}")
        print(f"Select an experiment to run (1-{len(experiments)}) or 'q' to quit:")
        user_input = input().strip()
        if user_input.lower() == 'q':
            print("Exiting.")
            exit()
        try:
            i = int(user_input) - 1
            if i < 0 or i >= len(experiments):
                print("Invalid experiment number. Please try again.")
                i = -1
                continue
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")
            i = -1
            continue

    # Refresh experiments from CSV before each experiment
    read_experiments_from_csv()
    exp = experiments[i]

    print("=" * 50)
    print(f"Starting experiment {i + 1}")
    print(f"Forward Speed: {exp['forward_linear_speed_cm_s']} cm/s at max {exp['forward_max_motor_rpm']} RPM")
    print(f"Backward Speed: {exp['backward_linear_speed_cm_s']} cm/s at max {exp['backward_max_motor_rpm']} RPM")

    trial = 0
    while True:
        trial += 1
        print("-" * 50)
        print(f"Trial {trial}")

        # Refresh experiments from CSV before forward movement
        read_experiments_from_csv()
        exp = experiments[i]

        # Forward
        print(f"Moving forward...")
        move(
            clockwise=False,
            wheel_diameter_cm=exp['wheel_diameter_cm'],
            target_travel_distance_cm=exp['target_travel_distance_cm'],
            target_linear_speed_cm_s=exp['forward_linear_speed_cm_s'],
            max_motor_rpm=exp['forward_max_motor_rpm']
        )

        print(f"Continue to backward movement? [Y/n]")
        if input().strip().lower() == 'n':
            break

        # Refresh experiments from CSV before backward movement
        read_experiments_from_csv()
        exp = experiments[i]

        # Backward
        print(f"Moving backward...")
        move(
            clockwise=True,
            wheel_diameter_cm=exp['wheel_diameter_cm'],
            target_travel_distance_cm=exp['target_travel_distance_cm'],
            target_linear_speed_cm_s=exp['backward_linear_speed_cm_s'],
            max_motor_rpm=exp['backward_max_motor_rpm']
        )

        print(f"Trial {trial} complete.")
        print(f"Continue to next trial? [Y/n]")
        if input().strip().lower() == 'n':
            break

    print(f"Experiment {i + 1} complete.")
    if i < len(experiments) - 1:
        print(f"Continue to next experiment? [Y/n]")
        if input().strip().lower() == 'n':
            break
