import csv
import time
import pybullet as p

# connect to the physics simulation
p.connect(p.GUI)

# Half the size of the landing pad
p_size = 2.12

# load objects
plane = p.loadURDF("urdf/plane.urdf")
vshape_side1 = p.loadURDF("urdf/v_shape.urdf", [p_size,0,0])
vshape_side2 = p.loadURDF("urdf/v_shape.urdf", [-p_size,0,0])
vshape_side3 = p.loadURDF("urdf/v_shape.urdf", [0,p_size,0], p.getQuaternionFromEuler([0,0,1.5708]))
vshape_side4 = p.loadURDF("urdf/v_shape.urdf", [0,-p_size,0], p.getQuaternionFromEuler([0,0,1.5708]))
xshape = p.loadURDF("urdf/x_shape.urdf", [0,0,3])

# enable real-time simulation and set gravity
p.setRealTimeSimulation(1)
p.setGravity(0,0,-10)

# Create or open a CSV file
file = open('xshape_position_orientation.csv', 'w', newline='')
writer = csv.writer(file)

# Write header
writer.writerow(["Attempt", "PosX", "PosY", "PosZ", "OrnX", "OrnY", "OrnZ", "OrnW"])

attempt = 0

# Half the size of the testing area
x_start=-1.8
y_start=-1.8
x=x_start
y=y_start


try:
    while True:
        pos, orn = p.getBasePositionAndOrientation(xshape)
        linear_vel, angular_vel = p.getBaseVelocity(xshape)

        # check if both linear and angular velocity are close to zero
        if max(abs(v) for v in linear_vel) < 0.01 and max(abs(v) for v in angular_vel) < 0.01:
            # if it is stable, move it to (0,0,5)
            p.resetBasePositionAndOrientation(xshape, [x,y,3], p.getQuaternionFromEuler([0,0,0]))
            attempt += 1
            if(x > -x_start):
                x = x_start
                y += 0.2
            else:
                x += 0.2
            if(y > -y_start):
                break
            print(attempt,x,y)

        writer.writerow([attempt] + list(pos) + list(orn))  # Write the attempt number and position and orientation values into the CSV file
        
        # print("Attempt: ", attempt)
        # print("Position of xshape: ", pos)
        # print("Orientation of xshape: ", orn)
        time.sleep(1./240.)
        
    print("Stopping the simulation...")
    p.disconnect()
    file.close()
except KeyboardInterrupt:
    print("Stopping the simulation...")
    p.disconnect()
    file.close()
