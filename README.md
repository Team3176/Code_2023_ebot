# Team 3176 2023 Robot

## Beta 2024

This version has been checked with wpilib beta 2 available [here](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1-beta-2)

until photonvision releases its beta the library must be downloaded seperatly and compiled. clone photonvision from [here](https://github.com/PhotonVision/photonvision)

from within the photonvision repo build and install the library to local maven

```
./gradlew generateVendorJson
./gradlew publishToMavenLocal

```

you may need to replace the photonvision.json in the vendor deps with the generated one in `photon-lib/build/generated/vendordeps/photonlib.json`

Finally open the robot project with the FRC 2024 VS Code and build

To run for simulation `Run Simulation` and select `Sim GUI` when it pops up

launch advantageScope and under `Help` click `Show Preferences...`. Then select for Live Source `NetworkTables 4 (AdvantageKit)`

Now you should be able to go `File` and `Connect to Simulator`

Welcome to the 3176 Robot for 2023 FRC Charged Up!

Features:

- Command Based Design
- LEDs State Communication
- Position Controlled Arm
- Custom SwervePod Support
- PathPlanner Autonomous Routines

