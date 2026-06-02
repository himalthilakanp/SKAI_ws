<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com) 
     Commit Version: 1.6.0-4-g7f85cfe  Build Version: 1.6.7995.38578
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot
  name="skai_arm">
  <link
    name="BASE">
    <inertial>
      <origin
        xyz="-0.064016 0.11562 0.080031"
        rpy="0 0 0" />
      <mass
        value="1.1031" />
      <inertia
        ixx="0.0023261"
        ixy="-1.4598E-06"
        ixz="-0.0001122"
        iyy="0.0024963"
        iyz="-5.3023E-06"
        izz="0.0023583" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/BASE.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.7451 0.73725 0.72941 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/BASE.STL" />
      </geometry>
    </collision>
  </link>
  <link
    name="J_1">
    <inertial>
      <origin
        xyz="0.0030632 -0.00037054 0.19009"
        rpy="0 0 0" />
      <mass
        value="1.089" />
      <inertia
        ixx="0.0018485"
        ixy="-1.4598E-06"
        ixz="-0.00011196"
        iyy="0.0024341"
        iyz="-5.3023E-06"
        izz="0.001692" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_1.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.7451 0.73725 0.72941 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_1.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="ROT_1"
    type="revolute">
    <origin
      xyz="-0.067 -0.065 0.082"
      rpy="0 0 0" />
    <parent
      link="BASE" />
    <child
      link="J_1" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-3.14"
      upper="3.14"
      effort="10"
      velocity="0.1" />
  </joint>
  <link
    name="J_2">
    <inertial>
      <origin
        xyz="-0.25856 0.13854 0.13905"
        rpy="0 0 0" />
      <mass
        value="0.721" />
      <inertia
        ixx="0.0019755"
        ixy="0.0020808"
        ixz="-3.9585E-06"
        iyy="0.004707"
        iyz="1.0786E-06"
        izz="0.00632" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_2.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.7451 0.73725 0.72941 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_2.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="PITCH_1"
    type="revolute">
    <origin
      xyz="0 -0.046 0.194"
      rpy="1.5708 1.2217 3.1416" />
    <parent
      link="J_1" />
    <child
      link="J_2" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-1.57"
      upper="1.57"
      effort="10"
      velocity="0.1" />
  </joint>
  <link
    name="J_3">
    <inertial>
      <origin
        xyz="0.15412 -0.0055136 0.088982"
        rpy="0 0 0" />
      <mass
        value="0.21768" />
      <inertia
        ixx="0.0012955"
        ixy="7.1563E-06"
        ixz="-0.0018514"
        iyy="0.0047078"
        iyz="4.1274E-06"
        izz="0.0034333" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_3.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.7451 0.73725 0.72941 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_3.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="PITCH_2"
    type="revolute">
    <origin
      xyz="-0.37152 0.2145 0.107"
      rpy="1.5708 0 0.87266" />
    <parent
      link="J_2" />
    <child
      link="J_3" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="-3.14"
      upper="3.14"
      effort="10"
      velocity="0.1" />
  </joint>
  <link
    name="J_4">
    <inertial>
      <origin
        xyz="0.0015946 0.0041989 -0.028525"
        rpy="0 0 0" />
      <mass
        value="0.2071" />
      <inertia
        ixx="0.00034764"
        ixy="1.6788E-07"
        ixz="-4.447E-08"
        iyy="0.00034785"
        iyz="-2.7415E-07"
        izz="0.00030028" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_4.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_4.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="PITCH_3"
    type="revolute">
    <origin
      xyz="0.3542 0.012 0.2045"
      rpy="1.5708 0.68579 0" />
    <parent
      link="J_3" />
    <child
      link="J_4" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-6.28"
      upper="6.28"
      effort="10"
      velocity="0.1" />
  </joint>
  <link
    name="J_5">
    <inertial>
      <origin
        xyz="-0.010591 -1.028232 -0.053684"
        rpy="0 0 0" />
      <mass
        value="0.26391" />
      <inertia
        ixx="0.00046898"
        ixy="-7.9386E-06"
        ixz="-1.1352E-05"
        iyy="0.00045081"
        iyz="-2.9949E-05"
        izz="0.00049256" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_5.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_5.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="ROT_2"
    type="revolute">
    <origin
      xyz="0.023514 0.062204 -0.0725"
      rpy="-1.5708 0.3614 2.7802" />
    <parent
      link="J_4" />
    <child
      link="J_5" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-6.28"
      upper="6.28"
      effort="10"
      velocity="0.1" />
  </joint>
  <link
    name="J_6">
    <inertial>
      <origin
        xyz="1.5103E-05 1.3545E-05 -0.26408"
        rpy="0 0 0" />
      <mass
        value="0.14576" />
      <inertia
        ixx="0.00029182"
        ixy="8.1929E-07"
        ixz="-6.0976E-08"
        iyy="0.00028743"
        iyz="-9.8159E-09"
        izz="0.00011651" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_6.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.79216 0.81961 0.93333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/J_6.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="ROT_3"
    type="revolute">
    <origin
      xyz="-0.12262 -0.3244 -0.0725"
      rpy="1.5708 -0.69813 -0.3614" />
    <parent
      link="J_5" />
    <child
      link="J_6" />
    <axis
      xyz="0 0 -1" />
    <limit
      lower="-6.28"
      upper="6.28"
      effort="10"
      velocity="0.1" />
  </joint>
</robot>