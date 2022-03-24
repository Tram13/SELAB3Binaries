# SEL3 Binary

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
                <base_module length="3.0"/>
            </geometry>
        </visual>
    </link>

    <!--Joints are allowed to be out of order-->
    <joint name="MODULE_1_JOINT" type="revolute">
        <!--For each parent, there should be at most one joint-->
        <parent link="anchor"/>
        <!--Linking an earlier element here will cause infinite recursion-->
        <child link="MODULE_1"/>
        <!--axis and dof are optional elements-->
        <axis xyz="0 1 0"/>
        <dof minAngle="-170.0" maxAngle="170.0"/>
    </joint>
</robot>
```

### Target side channel

GUID: 26013736-a87f-4c0c-b0ee-fdce6b3b45ff

Expected format: Vector3

`<x> <y> <z>`

### Obstacle side channel

GUID: c5b83134-3b07-4f91-8edb-44c26e890eaf

Expected format: URDF

```xml
<?xml version="1.0"?>
<obstacles name="Obstacle_description_v1">

    <!-- T-shaped bar at (0 0 10)-->
    <obstacle object_type="Static_Pillar" location="0 0 10"/>
    <obstacle object_type="Static_Pillar" location="0 10 10" rotation="0 0 90"/>

    <!-- Box around (0 5 -10)-->
    <obstacle object_type="Static_Wall" location="0 5 -15" rotation="0 0 0"/>
    <obstacle object_type="Static_Wall" location="5 5 -10" rotation="0 90 0"/>
    <obstacle object_type="Static_Wall" location="0 5 -5" rotation="0 0 0"/>
    <obstacle object_type="Static_Wall" location="-5 5 -10" rotation="0 90 0"/>

</obstacles>
```
