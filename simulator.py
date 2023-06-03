import csv
import time
import pybullet as p
import pybullet_data
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from datetime import datetime

def process_csv(filename):
    # Read the CSV file
    df = pd.read_csv(filename)
    
    # Find the minimum time where result is "Successful"
    min_time_success = df[df['Result'] == 'Successful']['Time (s)'].min()
    
    # Replace the time where result is "Unsuccessful" with the minimum time
    df.loc[df['Result'] == 'Unsuccessful', 'Time (s)'] = min_time_success

    # Write back the modified DataFrame to the CSV file
    df.to_csv(filename, index=False)

def create_plot(f_name):
    # Set the font family to Times New Roman
    plt.rcParams['font.family'] = 'Times New Roman'

    # Load the data from your CSV file
    df = pd.read_csv(f_name)

    # Select only the columns 'StartX', 'StartY', 'Time (s)', 'Result' from the dataframe
    df_selected = df[['StartX', 'StartY', 'Time (s)', 'Result']].copy()

    unique_Results = df_selected['Result'].nunique()
    cmap = sns.cubehelix_palette(rot=.2,dark=.25, light=.5, n_colors=unique_Results, reverse=True)

    # Specify the order in which you want to present the categories
    hue_order = ['Successful', 'Unsuccessful'] 

    # Create the jointplot
    ax = sns.relplot(data=df_selected, x="StartX", y="StartY", hue="Result", hue_order=hue_order, size="Time (s)", palette=cmap, sizes=(10, 100))

    min_time = df_selected["Time (s)"].min()
    max_time = df_selected["Time (s)"].max()

    print(f'Time range: {min_time} - {max_time} seconds')

    # Set x and y labels
    plt.xlabel('X Axis (m)')
    plt.ylabel('Y Axis (m)')

    # Get the current timestamp
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

    # Construct the file name with the timestamp
    file_name = f'plot/plot_{f_name + timestamp}.png'

    # Increase the quality (resolution) of the plot when saving as an image file
    plt.savefig(file_name, dpi=300)

    # Show the plot
    # plt.show()

# Checking base sizes (m)
base_sizes= [3]

# Checking angle (degree)
angles = [30, 45, 60, 75]

resolution = 0.250

# gravity
g = 9.80

# Drone height 
d_height = 0.4
# Drone landing pad height 
l_pad_height = 0.05

# Sleep time (s) *don't set below .005
t_sleep = 0.005
print(t_sleep)

# Connect to the physics simulation
p.connect(p.GUI)
# Collect PyBullet default data path.
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Enable real-time simulation and set gravity
p.setRealTimeSimulation(1)
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
drone = p.loadURDF("urdf/drone.urdf", [max(base_sizes), max(base_sizes), 1], p.getQuaternionFromEuler([0, 0, 0]))

# Get Drone center hight at Z=0
time.sleep(2)
pos, orn = p.getBasePositionAndOrientation(drone)
z_drone_center_hight = pos[2]

for base_size in base_sizes:
    # Calculate grid size
    x_start = -(base_size/2) - resolution/2
    y_start = -(base_size/2) - resolution/2
        
    x_end = (base_size/2) + resolution/2
    y_end = (base_size/2) + resolution/2

    for angle in angles:

        # Drop height
        # drop_h = d_height + max(base_sizes)/(2*np.tan(np.radians(min(angles))))
        drop_h = d_height + base_size/(2*np.tan(np.radians(min(angles))))

        radian = np.radians(angle)
        # Calculate base height
        base_height = base_size/(2*np.tan(radian))

        # Reset Values
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
        
        p.resetBasePositionAndOrientation(drone, [x, y, drop_h], p.getQuaternionFromEuler([0, 0, 0.785398]))
        
        # Create a CSV file Success Attempt Record
        file_name = f"xshape_success_base_{base_size}_angle_{angle}.csv"
        file = open(file_name, 'w', newline='')

        writer_s = csv.writer(file)

        # Header: Success Attempt Record
        writer_s.writerow(["StartX", "StartY", "Time (s)", "Result"])
        
        while True:

            pos, orn = p.getBasePositionAndOrientation(drone)
            linear_vel, angular_vel = p.getBaseVelocity(drone)

            # check if both linear and angular velocity are close to zero
            if max(abs(v) for v in linear_vel) < 0.005 and max(abs(v) for v in angular_vel) < 0.01:

                time.sleep(1) # add 1 s delay for more stable
                pos, orn = p.getBasePositionAndOrientation(drone) # update position
                # Check if the x and y position values are close to zero
                if max(abs(val) for val in list(pos[:2])) < 0.03 and pos[2] > z_drone_center_hight + l_pad_height/2:
                    Result = 'Successful'
                    l_time = l_count * t_sleep
                else:
                    Result = 'Unsuccessful'
                    l_time = l_count * t_sleep
                
                # update x, y position
                print("Base size:", base_size, " Angle:", angle," X:", round(x, 3), " Y:", round(y, 3), " Result:", Result)

                # Write the successful attempt record into the CSV file.
                writer_s.writerow([ round(x, 3)] + [round(y, 3)] + [round(l_time, 4)] + [Result])
                
                # loop count reset
                l_count = 0 

                if (x >= x_end ):
                    x = x_start
                    y += resolution
                else:
                    x += resolution

                x = round(x, 3)
                y = round(y, 3)

                if (y > y_end ):
                    break
                
                # if it is stable, move it to (x,y,Drop height)
                p.resetBasePositionAndOrientation(drone, [x, y, drop_h], p.getQuaternionFromEuler([0, 0, 0.785398]))

            else:
                l_count += 1

            time.sleep(t_sleep)
        # close file
        file.close()
        process_csv(file_name)
        create_plot(file_name)

print("Stopping the simulation...")
p.disconnect()