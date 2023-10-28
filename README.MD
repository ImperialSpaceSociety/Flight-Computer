# Imperial College Space Society - High-power Rocketry

## Rocket

The rocket was initially designed in a program called OpenRocket and then the particular components were designed in Fusion 360 and manufactured using 3D printers or a laser cutter.

![Rocket](./Assets/Rocket.jpg)

## Flight computer

The flight computer is based on STM32 platform and the main diagram with its components can be seen below.

![Flight computer diagram](./Assets/Diagram.jpg)

The following picture shows the 3D render of the flight computer designed in KiCAD.

![Flight computer diagram](./Assets/PCB.png)

## Launch

The rocket was successfully launched and recovered on 9th July 2023 in Twycross, Leicestershire. The rocket reached the altitude of 721 m after  s of flight.

![Flight computer diagram](./Assets/Altitude.png)

## Future improvements

- UART connection 
- Button/jumper pin header for reset
- Code for magnetometer and flashchip
- FreeRTOS
- Datalog in MATLAB such that the incoming data is being processed and then saved

## Repository content

- `Assets` - This folder contains images and plots for the `README.md` file.
- `Rocket` - This folder contains files simulation and design files for the rocket.
- `Hardware/circuit board design v1.0` - First version of the hardware files of the flight computer.
- `Hardware/circuit board design v2.1` - First version of the hardware files of the flight computer.
- `References` - This folder contains datasheets and manuals relevant to the components used for the flight computer.
- `Software/Dev board code` - This folder contains projects used to develop and test libraries for each of the components of the computer.
- `Software/fcs-v2` - Second version of the flight computer software.
- `Software/revision_1` - First version of the flight computer software.

## Team

- Awais Khawaja (Rocket team lead)
- Rohan Kukreti, Stephen Liu (Nose cone)
- Heather Boatin (Recovery system)
- Denzel Goh, Dongxiao Jiang (Fins)
- Jack Crane (Engine mount)
- Timothy Newman, Lou Alsteens, and Jakub J\c edrzejewski (PCB)
- Václav Pavlíček, Richard Hu, Benny Liu (Software)

## Acknowledgements

We would like to thank Victor Boddy from the Electrical Engineering Undergraduate lab and the Advanced Hackspace staff for their help and support.