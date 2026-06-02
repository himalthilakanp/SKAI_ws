<?xml version="1.0" encoding="utf-8"?>
<!-- This URDF was automatically created by SolidWorks to URDF Exporter! Originally created by Stephen Brawner (brawner@gmail.com)
     Commit Version: 1.6.0-4-g7f85cfe  Build Version: 1.6.7995.38578
     For more information, please see http://wiki.ros.org/sw_urdf_exporter -->
<robot
  name="Assem1.SLDASM">
  <link
    name="base_link">
    <inertial>
      <origin
        xyz="-4.44089209850063E-16 -1.11022302462516E-16 0.0135526315789474"
        rpy="0 0 0" />
      <mass
        value="1.3318" />
      <inertia
        ixx="0.00492995374464995"
        ixy="6.26387758632831E-36"
        ixz="-4.18812285385323E-36"
        iyy="0.00492995374464995"
        iyz="5.809331418274E-19"
        izz="0.00968555469470994" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/base_link.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/base_link.STL" />
      </geometry>
    </collision>
  </link>
  <link
    name="Rot_1">
    <inertial>
      <origin
        xyz="0 0 0.00604866039614907"
        rpy="0 0 0" />
      <mass
        value="0.746929158456676" />
      <inertia
        ixx="0.00234165144087209"
        ixy="-7.44462551297724E-21"
        ixz="1.57537802825337E-19"
        iyy="0.00233239070927305"
        iyz="6.54398656559878E-20"
        izz="0.00328798834831709" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Rot_1.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Rot_1.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="J_1"
    type="revolute">
    <origin
      xyz="0 0 0.045"
      rpy="0 0 0" />
    <parent
      link="base_link" />
    <child
      link="Rot_1" />
    <axis
      xyz="0 0 1" />
    <limit
      lower="-1.57"
      upper="1.57"
      effort="3"
      velocity="0.1" />
  </joint>
  <link
    name="Link_1">
    <inertial>
      <origin
        xyz="0.2725 -1.11022302462516E-16 0"
        rpy="0 0 0" />
      <mass
        value="2.09442476288859" />
      <inertia
        ixx="0.00196639858335801"
        ixy="6.90794779257081E-18"
        ixz="3.89079898761808E-18"
        iyy="0.0661896996602056"
        iyz="2.5293262131739E-20"
        izz="0.0646509127992707" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Link_1.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Link_1.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="J_2"
    type="revolute">
    <origin
      xyz="0 0 0.05"
      rpy="0 0 0" />
    <parent
      link="Rot_1" />
    <child
      link="Link_1" />
    <axis
      xyz="0 -1 0" />
    <limit
      lower="0"
      upper="1.57"
      effort="3"
      velocity="0.1" />
  </joint>
  <link
    name="Link_2">
    <inertial>
      <origin
        xyz="0.158520152581401 0 -1.52655665885959E-16"
        rpy="0 0 0" />
      <mass
        value="1.16160773230196" />
      <inertia
        ixx="0.00118353197585008"
        ixy="1.38573436284514E-18"
        ixz="2.63751628639377E-19"
        iyy="0.00825133506001472"
        iyz="1.93571952705547E-20"
        izz="0.00749584989243868" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Link_2.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/Link_2.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="J_3"
    type="revolute">
    <origin
      xyz="0.545 0 0"
      rpy="0 0 0" />
    <parent
      link="Link_1" />
    <child
      link="Link_2" />
    <axis
      xyz="0 1 0" />
    <limit
      lower="-1.57"
      upper="1.57"
      effort="0"
      velocity="0.1" />
  </joint>
  <link
    name="EE">
    <inertial>
      <origin
        xyz="-0.00540353607903965 0 -8.32667268468867E-16"
        rpy="0 0 0" />
      <mass
        value="0.0391605143576071" />
      <inertia
        ixx="3.60041652275502E-05"
        ixy="3.98383761530666E-20"
        ixz="4.44393152097124E-20"
        iyy="3.06693674510865E-05"
        iyz="-8.94278483786476E-12"
        izz="5.94457547492915E-06" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/ee.STL" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
    <collision>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://arm_rviz/meshes/ee.STL" />
      </geometry>
    </collision>
  </link>
  <joint
    name="EE"
    type="fixed">
    <origin
      xyz="0.3 0 0"
      rpy="0 0 0" />
    <parent
      link="Link_2" />
    <child
      link="EE" />
    <axis
      xyz="0 0 0" />
  </joint>
</robot>
