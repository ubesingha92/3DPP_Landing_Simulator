# 3D Passive Positioning Landing Simulator

This project contains a simulation that uses the PyBullet physics engine to simulate passive positioning landing of different 3D shapes. The shapes are placed in a 3D environment, and their positions and orientations are tracked over time. The collected data is stored in a CSV file for further analysis.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

# Prerequisites

- **Python:** You need to have Python 3.x installed on your machine. If you don't have Python installed, you can download it from [here](https://www.python.org/downloads/).

- **Microsoft Visual C++ 14.0:** This is required to build certain Python packages. For Windows users, download and install "Build Tools for Visual Studio 2019" from [Microsoft's official site](https://visualstudio.microsoft.com/downloads/). 

Note: The installation process can take a while as the installer will need to download more than 1 GB of additional files.

### Dependencies

The simulation uses PyBullet for the physics simulation. You can install them using pip:

```
pip install pybullet
```

### URDF Files

The project uses URDF (Unified Robot Description Format) files to describe the 3D shapes used in the simulation. Make sure the following URDF files are in your project directory:

- "urdf/v_shape.urdf"
- "urdf/x_shape.urdf"

### Running the Simulation

Simply navigate to the directory where the script is located and run the script using Python:

```
python simulator.py
```
![land interface](https://github.com/ubesingha92/3DPP_Landing_Simulator/assets/126043311/8edc58be-abf6-4e4c-bf80-a30821298fe2)


The simulation will then run until you manually stop it.

## Data Output

The script outputs a CSV file named 'xshape_position_orientation.csv' that records the attempt number, position and orientation of the 'x_shape' at each step of the simulation.

## Stopping the Simulation

To stop the simulation, simply press `Ctrl+C` in the terminal where the script is running.

## Authors

- Chanaka Chathuranga Ubesingha

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
