import csv
import time
import pybullet as p
import pybullet_data

# Half the size of the landing pad
p_size = 1.5

# Half the size of the testing area
x_start = -2.6
y_start = -2.6
z_start = 3
x = x_start
y = y_start
z = z_start

attempt = 0 # attempt number
massage = ''
t_sleep = 0.01
l_count = 0.0 # loop count in a single attempt

p_update = 0.2

# connect to the physics simulation
p.connect(p.GUI)
# Collect PyBullet default data path.
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# enable real-time simulation and set gravity
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)

# load objects
planeId = p.loadURDF("plane.urdf")

vshape_side1 = p.loadURDF("urdf/v_shape.urdf", [p_size, p_size, 0], p.getQuaternionFromEuler([0, 0, 0.785398]))
vshape_side1 = p.loadURDF("urdf/v_shape.urdf", [p_size, -p_size, 0], p.getQuaternionFromEuler([0, 0, 2.35619]))
vshape_side1 = p.loadURDF("urdf/v_shape.urdf", [-p_size, p_size, 0], p.getQuaternionFromEuler([0, 0, 2.35619]))
vshape_side1 = p.loadURDF("urdf/v_shape.urdf", [-p_size, -p_size, 0], p.getQuaternionFromEuler([0, 0, 0.785398]))

xshape = p.loadURDF("urdf/x_shape.urdf", [x, y, z], p.getQuaternionFromEuler([0, 0, 0]))

# Create a CSV file Success Attempt Record
file = open('xshape_success.csv', 'w', newline='')
writer_s = csv.writer(file)

# Create a CSV file Attempt Number, Position, and Orientation Values
file = open('xshape_position_orientation.csv', 'w', newline='')
writer_op = csv.writer(file)

# Header: Attempt Number, Position, and Orientation Values
writer_op.writerow(["Attempt", "time(s)", "PosX", "PosY", "PosZ","OrnX", "OrnY", "OrnZ", "OrnW"])

# Header: Success Attempt Record
writer_s.writerow(["Attempt", "StartX", "StartY", "StartZ", "time(s)", "Massage"])

try:
    while True:
        pos, orn = p.getBasePositionAndOrientation(xshape)
        linear_vel, angular_vel = p.getBaseVelocity(xshape)

        # Write the attempt number and position and orientation values into the CSV file
        writer_op.writerow([attempt] + [l_count * t_sleep] + [round(val, 2) for val in list(pos) + list(orn)])

        # check if both linear and angular velocity are close to zero
        if max(abs(v) for v in linear_vel) < 0.01 and max(abs(v) for v in angular_vel) < 0.01:

            # Check if the object's position values are also close to zero
            if max(abs(val) for val in list(pos)) < 0.4:
                print('OK')
                massage = 'OK'
            else:
                print('NG')
                massage = 'NG'

            # Write the successful attempt record into the CSV file.
            writer_s.writerow([attempt] + [x] + [y] + [z] + [l_count * t_sleep] + [massage])
            
            # if it is stable, move it to (x,y,3)
            p.resetBasePositionAndOrientation(xshape, [x, y, 3], p.getQuaternionFromEuler([0, 0, 0]))
            
            attempt += 1 # update attempt count
            l_count = 0 # loop count reset

            # update x, y position
            print(attempt, round(x, 2), round(y, 2))
            if (x > -x_start):
                x = x_start
                y += p_update
            else:
                x += p_update
            if (y > -y_start):
                break

        l_count += 1
        time.sleep(t_sleep)

    print("Stopping the simulation...")
    p.disconnect()
    file.close()
except KeyboardInterrupt:
    print("Stopping the simulation...")
    p.disconnect()
    file.close()
