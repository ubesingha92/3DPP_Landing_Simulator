import csv
import time
import pybullet as p
import pybullet_data
import numpy as np

# Checking base sizes (m)
base_sizes= [4, 5, 6, 7, 8]

# Checking angle (degree)
angles = [45, 60, 75]

# Checking grid spacing (m)
resolution = 1

# gravity
g = 9.80

# Drone height 
d_height = 0.41

# Drop height
drop_h = d_height + max(base_sizes)/(2*np.tan(np.radians(min(angles))))

# Sleep time (s)
t_sleep = .01

# Connect to the physics simulation
p.connect(p.GUI)
# Collect PyBullet default data path.
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Enable real-time simulation and set gravity
p.setRealTimeSimulation(1)
# Set gravity (m/s²)
p.setGravity(0, 0, -g)

# Load objects
planeId = p.loadURDF("plane.urdf")

# Base
shape_side1_p1 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
shape_side1_p2 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

shape_side2_p1 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
shape_side2_p2 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

shape_side3_p1 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
shape_side3_p2 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

shape_side4_p1 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))
shape_side4_p2 = p.loadURDF("urdf/I_shape.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

# Drone
drone = p.loadURDF("urdf/drone.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))

for base_size in base_sizes:

    for angle in angles:

        radian = np.radians(angle)
        # Calculate base height
        base_height = base_size/(2*np.tan(radian))

        # Calculate grid size
        x_start = -(base_size/2)
        y_start = -(base_size/2)
        
        x_end = (base_size/2)
        y_end = (base_size/2)

        x = x_start
        y = y_start

        # How much time does it take for the drone to land
        l_time =0
        # Initialize loop count
        l_count = 0

        # Base Position
        p.resetBasePositionAndOrientation(shape_side1_p1, [base_size/2, base_size/2, base_height], p.getQuaternionFromEuler([-radian, 0, 0]))
        p.resetBasePositionAndOrientation(shape_side1_p2, [base_size/2, base_size/2, base_height], p.getQuaternionFromEuler([0, radian, 0]))

        p.resetBasePositionAndOrientation(shape_side2_p1, [base_size/2, -base_size/2, base_height], p.getQuaternionFromEuler([radian, 0, 0]))
        p.resetBasePositionAndOrientation(shape_side2_p2, [base_size/2, -base_size/2, base_height], p.getQuaternionFromEuler([0, radian, 0]))

        p.resetBasePositionAndOrientation(shape_side3_p1, [-base_size/2, base_size/2, base_height], p.getQuaternionFromEuler([-radian, 0, 0]))
        p.resetBasePositionAndOrientation(shape_side3_p2, [-base_size/2, base_size/2, base_height], p.getQuaternionFromEuler([0, -radian, 0]))

        p.resetBasePositionAndOrientation(shape_side4_p1, [-base_size/2, -base_size/2, base_height], p.getQuaternionFromEuler([radian, 0, 0]))
        p.resetBasePositionAndOrientation(shape_side4_p2, [-base_size/2, -base_size/2, base_height], p.getQuaternionFromEuler([0, -radian, 0]))
        
        time.sleep(1)
        p.resetBasePositionAndOrientation(drone, [x, y, drop_h], p.getQuaternionFromEuler([0, 0, 0.785398]))
        
        # Create a CSV file Success Attempt Record
        file_name = f"xshape_success_base_{base_size}_angle_{angle}.csv"
        file = open(file_name, 'w', newline='')

        writer_s = csv.writer(file)

        # Header: Success Attempt Record
        writer_s.writerow(["StartX", "StartY", "Time (s)", "Result"])
                
        while x <= x_end and y <= y_end:

            pos, orn = p.getBasePositionAndOrientation(drone)
            linear_vel, angular_vel = p.getBaseVelocity(drone)

            # check if both linear and angular velocity are close to zero
            if max(abs(v) for v in linear_vel) < 0.001 and max(abs(v) for v in angular_vel) < 0.001:

                # Check if the x and y position values are close to zero
                if max(abs(val) for val in list(pos[:2])) < 0.1:
                    Result = 'Successful'
                    l_time = l_count * t_sleep
                else:
                    Result = 'Unsuccessful'
                    l_time = l_count * t_sleep
                
                # update x, y position
                print("Base size:", base_size, " Angle:", angle," X:", round(x, 2), " Y:", round(y, 2), " Result:", Result)

                # Write the successful attempt record into the CSV file.
                writer_s.writerow([ round(x, 2)] + [round(y, 2)] + [l_time] + [Result])
                
                # if it is stable, move it to (x,y,Drop height)
                p.resetBasePositionAndOrientation(drone, [x, y, drop_h], p.getQuaternionFromEuler([0, 0, 0.785398]))
                
                l_count = 0 # loop count reset

                # update x, y position
                # print("Base size:", base_size, " Angle:", angle," X:", round(x, 2), " Y:", round(y, 2), " Result:", Result)

                if (x >= x_end):
                    x = x_start
                    y += resolution
                else:
                    x += resolution
            else:
                l_count += 1

            time.sleep(t_sleep)
        # close file
        file.close()

print("Stopping the simulation...")
p.disconnect()