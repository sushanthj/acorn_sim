---
layout: page
title: Jamboard
permalink: /Jamboard/
nav_order: 3
---

# Notes

1. Build Simulation with simple 4 wheel robot
2. Add Twisted Fields controller to move the robot
3. Improve Robot Visuals (use CAD)
4. Improve Robot Surroundings (Google Maps overlay, real plants)

## Example Models and Worlds

1. [Simple 4 Wheel Robot](https://app.gazebosim.org/OpenRobotics/fuel/models/X2%20Config%207)
2. [Fancy Outdoor World](https://app.gazebosim.org/Penkatron/fuel/worlds/Rubicon%20World)
3. [MP400 URDF Setup](https://github.com/neobotix/neo_simulation2/blob/humble/robots/mp_400/mp_400.urdf)

## Process to Convert CAD to SDF and DAE

1. Design CAD and export entire assembly of chassis as one object (one SLDPRT not SLDASSY)
2. Convert the single SLDPRT to STL
3. Use Blender to convert STL to DAE (after adding any necessary colors)

Q. Is there a better way to do the DAE conversion without blender?
Ans. Yes, use FreeCAD to convert STL to DAE but that seems to be more painful

Here's a link which I found useful: [Convert CAD -> STL -> DAE](https://www.youtube.com/watch?v=zGWFojrPoSA)

**TODO:** Make a video of this for future reference

## Integrate DAE and SDF into Robot URDF and World SDF

