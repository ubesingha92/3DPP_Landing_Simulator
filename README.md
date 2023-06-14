# 3D Passive Positioning Landing Simulator

This project contains a simulation that uses the PyBullet physics engine to simulate the passive positioning landing of drones on different 3D shapes. The shapes are positioned on a grid in a 3D environment, and a drone attempts to land on different points on these shapes. The position, time taken, and result (successful or unsuccessful) for each landing attempt are tracked and stored in a CSV file for further analysis.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- **Python:** You need to have Python 3.x installed on your machine. If you don't have Python installed, you can download it from [here](https://www.python.org/downloads/).

- **Microsoft Visual C++ 14.0:** This is required to build certain Python packages. For Windows users, download and install "Build Tools for Visual Studio 2019" from [Microsoft's official site](https://visualstudio.microsoft.com/downloads/). 

Note: The installation process can take a while as the installer will need to download more than 1 GB of additional files.

### Dependencies

The simulation uses several Python libraries including PyBullet for the physics simulation, pandas for data manipulation, and seaborn for data visualization. You can install these using pip:

```
pip install pybullet pandas seaborn
```

### URDF Files

The project uses URDF (Unified Robot Description Format) files to describe the 3D shapes used in the simulation. Make sure the following URDF files are in your project directory:

- "urdf/I_shape.urdf"
- "urdf/drone.urdf"

### Running the Simulation

Navigate to the directory where the script is located and run the script using Python:

```
python drone_landing_simulation.py
```

The simulation will then run, cycling through various platform configurations and landing attempts.

## Data Output

The script outputs a CSV file for each combination of base size and angle parameters, recording the start X and Y positions, time taken, and result (successful or unsuccessful) for each landing attempt. The time for unsuccessful attempts is replaced by the minimum time of successful attempts.

## Visualization

After running all the landing attempts for a specific set of parameters, the data is processed and a plot is created, visualizing the landing attempts across the start positions with the point size representing the time taken and the color representing the success or failure of the attempt.

## Stopping the Simulation

To stop the simulation, simply press `Ctrl+C` in the terminal where the script is running.

## Authors

- Chanaka Chathuranga Ubesingha

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
