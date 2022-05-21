# Unity Environment

## Controls

+ Camera movement: arrow keys
+ Camera zoom: scroll wheel
+ Take screenshot: s

## Editor only controls

+ Select joint: z-x
+ Move selected joint: a-e; q-d; w-c
+ Reset joints: r

## Observation vector

For every joint:

+ Target angle
+ Current angle
+ Current joint posistion (x,y,z)
+ Collisions (Arm, Ground, Obstacle)

End effector:

+ Current end effector posistion (x,y,z)

For every sensor:

+ Distance to nearest obstacle, normalized by max range; If nothing is detected, distance is 1

## Sidechannels

### Creation side channel

GUID: f1833626-0e9f-4d26-a0d0-3b18ae010549

Expected format: URDF

```xml
<?xml version="1.0"?>
<robot name="robot_GENOME_ID">
    <!--Exactly one link with an anchor_module should exist-->
    <link name="anchor">
        <visual>
            <geometry>
                <anchor_module/>
            </geometry>
        </visual>
    </link>

    <!--Links are allowed to be out of order-->
    <!--Names should be unique-->
    <link name="MODULE_1">
        <visual>
            <geometry>
                <!--Base modules are straight modules with variable length-->
                <base_module length="3.0" rotation="0.0"/>
            </geometry>
        </visual>
    </link>

    <link name="MODULE_2">
        <visual>
            <geometry>
                <!--Hook modules contain a 90° bend with a rotation towards their parent-->
                <hook_module rotation="45.0"/>
            </geometry>
        </visual>
    </link>

    <link name="MODULE_3">
        <visual>
            <geometry>
                <!--Telescopic modules can extend up to their length, but do not have a revolute joint-->
                <telescopic_module length="2.0" rotation="0.0"/>
            </geometry>
        </visual>
    </link>

    <link name="MODULE_4">
        <visual>
            <geometry>
                <!--Short modules don't have a body, they allow multiple degrees of freedom between two module bodies-->
                <short_module rotation="0.0"/>
            </geometry>
        </visual>
    </link>

    <!--Joints are allowed to be out of order-->
    <joint name="MODULE_1_JOINT">
        <!--For each parent, there should be at most one joint-->
        <parent link="anchor"/>
        <!--Linking an earlier element here will cause infinite recursion-->
        <child link="MODULE_1"/>
        <!--axis and dof are optional elements-->
        <axis xyz="0 1 0"/>
        <!--minAngle and maxAngle should be multiples of stepSize-->
        <dof minAngle="-170.0" maxAngle="170.0" stepSize="5.0"/>
        <!--spring is optional, decides how stable the joint is-->
        <spring stiffness="1000000" damping="100000"/>
    </joint>

    <!--Sensors will be sorted by parent-->
    <sensor name="SENSOR_1" range="3.0">
        <parent link="MODULE_4"/>
        <!--axis is optional, default is 0 1 0-->
        <axis xyz="0 1 0"/>
    </sensor>
</robot>
```

### Set position side channel

GUID: 9066f9e9-6050-4198-b8a9-fa2330252ff6

Expected format: List of floats

`<p1> <p2> <p3> ...`

### Target side channel

GUID: 26013736-a87f-4c0c-b0ee-fdce6b3b45ff

Expected format: Vector3

`<x> <y> <z>`

Returns:

+ 200 (Success) if succeeded
+ 409 (Conflict) when the target overlaps with obstacles

### Workspace side channel

GUID: 34e11e85-f324-4d83-97dc-ca22c9043fd1

Expected format: Vector3

`<x> <y> <z>`

Alternate message: "Clear" (Clears display)

### Obstacle side channel

GUID: c5b83134-3b07-4f91-8edb-44c26e890eaf

Expected format: xml

```xml
<?xml version="1.0"?>
<obstacles name="Obstacle_description_v4">

    <!-- T-shaped bar at 0 0 10 -->
    <obstacle object_type="Container" location="0 0 10">
        <obstacle object_type="Pillar" location="0 3 0" scale="1 0.5 1"/>
        <obstacle object_type="Pillar" location="0 6 0" rotation="0 0 90"/>
    </obstacle>

    <!-- Box around 0 0 -10 -->
    <obstacle object_type="Container" location="0 0 -10">
        <!-- lerp allows for movement -->
        <lerp duration="10" target-rotation="0 90 0" mode="repeat"/>
        <obstacle object_type="Wall" location="0 3 -3"/>
        <obstacle object_type="Wall" location="3 3 0" rotation="0 90 0"/>
        <obstacle object_type="Wall" location="0 3 3"/>
        <obstacle object_type="Wall" location="-3 3 0" rotation="0 90 0"/>
    </obstacle>
</obstacles>
```

Alternate message: "Clear" (Clears all obstacles)

### Logging side channel

GUID: ee1c1c99-7540-474e-8938-656df7ec81de

Expected format: None

This sidechannel exfiltrates any Debug message created by the Unity environment

### Snapshot side channel

GUID: 43ea94d6-2d2b-4661-b7cd-df5bc998793f

Expected format: Filename

Note: make sure the folder you are writing to exists!

### Text display side channel

GUID: 3f64a11e-480a-444d-8dca-15be95f1f7ee

Expected format: Plain text

Displays plain text in top right corner
