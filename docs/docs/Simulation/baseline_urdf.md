---
layout: default
title: Baseline URDF
parent: Simulation
nav_order: 2
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>


## Introduction

Let's build a simple 4 wheel robot in a modular fashion without worrying about the visual
aspects. We will use the URDF format to define the robot and the world.

# Baseline URDF

## Achieving Modular Design

- XACRO is a macro for URDF which allows us to import child URDFs into a parent URDF.
- In our application, I wanted the wheel module to be a separate URDF which can be imported into the main URDF.
- Here's a simple example of a wheel module:

```xml
<?xml version="1.0"?>

<robot name="wheel" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="wheel" params="name">
    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
    </link>
    <joint name="${name}_joint" type="continuous">
      <parent link="${name}_link"/>
      <child link="${name}_wheel"/>
      <origin xyz="0 0 0.05"/>
      <axis xyz="0 0 1"/>
    </joint>
    <link name="${name}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

</robot>
```

- The `wheel` macro defines a wheel with a cylinder link and a joint to connect the wheel to the robot.
- The `wheel` macro can be imported into the main URDF using the following syntax:

```xml
<?xml version="1.0"?>
<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find my_robot_description)/urdf/wheel.urdf.xacro"/>

  <xacro:wheel name="front_left"/>
  <xacro:wheel name="front_right"/>
  <xacro:wheel name="rear_left"/>
  <xacro:wheel name="rear_right"/>

</robot>
```