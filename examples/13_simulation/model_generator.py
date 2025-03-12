#!/usr/bin/env python3

import os
import argparse
import xml.dom.minidom
import numpy as np

class URDFModelGenerator:
    """
    URDF模型生成器，用于生成简单的机器人模型
    """
    def __init__(self):
        self.template_header = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot_name}">

'''
        
        self.template_footer = '''
</robot>'''
        
        self.material_template = '''  <material name="{name}">
    <color rgba="{r} {g} {b} {a}"/>
  </material>
'''
        
        self.link_template = '''  <link name="{name}">
    <visual>
      <geometry>
        {geometry}
      </geometry>
      <material name="{material}"/>
    </visual>
    <collision>
      <geometry>
        {geometry}
      </geometry>
    </collision>
    <inertial>
      <mass value="{mass}"/>
      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}"/>
    </inertial>
  </link>
'''
        
        self.joint_template = '''  <joint name="{name}" type="{type}">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{x} {y} {z}" rpy="{roll} {pitch} {yaw}"/>
    {axis}
    {limits}
  </joint>
'''
        
        self.axis_template = '''<axis xyz="{x} {y} {z}"/>'''
        
        self.limits_template = '''<limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{velocity}"/>'''
        
        self.gazebo_plugin_template = '''  <gazebo>
    <plugin name="{name}" filename="{filename}">
      {content}
    </plugin>
  </gazebo>
'''
        
        self.sensor_template = '''  <gazebo reference="{link}">
    <sensor name="{name}" type="{type}">
      {properties}
      <plugin name="{plugin_name}" filename="{plugin_filename}">
        {plugin_content}
      </plugin>
    </sensor>
  </gazebo>
'''
    
    def generate_box_link(self, name, size, mass, material, origin=None):
        """生成立方体链接"""
        x, y, z = size
        geometry = f'<box size="{x} {y} {z}"/>'
        
        # 计算惯性矩
        ixx = mass/12.0 * (y*y + z*z)
        iyy = mass/12.0 * (x*x + z*z)
        izz = mass/12.0 * (x*x + y*y)
        
        return self.link_template.format(
            name=name,
            geometry=geometry,
            material=material,
            mass=mass,
            ixx=ixx,
            ixy=0,
            ixz=0,
            iyy=iyy,
            iyz=0,
            izz=izz
        )
    
    def generate_cylinder_link(self, name, radius, length, mass, material):
        """生成圆柱体链接"""
        geometry = f'<cylinder radius="{radius}" length="{length}"/>'
        
        # 计算惯性矩
        ixx = mass/12.0 * (3*radius*radius + length*length)
        iyy = mass/12.0 * (3*radius*radius + length*length)
        izz = mass/2.0 * radius*radius
        
        return self.link_template.format(
            name=name,
            geometry=geometry,
            material=material,
            mass=mass,
            ixx=ixx,
            ixy=0,
            ixz=0,
            iyy=iyy,
            iyz=0,
            izz=izz
        )
    
    def generate_sphere_link(self, name, radius, mass, material):
        """生成球体链接"""
        geometry = f'<sphere radius="{radius}"/>'
        
        # 计算惯性矩
        inertia = 2.0/5.0 * mass * radius * radius
        
        return self.link_template.format(
            name=name,
            geometry=geometry,
            material=material,
            mass=mass,
            ixx=inertia,
            ixy=0,
            ixz=0,
            iyy=inertia,
            iyz=0,
            izz=inertia
        )
    
    def generate_fixed_joint(self, name, parent, child, xyz, rpy):
        """生成固定关节"""
        x, y, z = xyz
        roll, pitch, yaw = rpy
        
        return self.joint_template.format(
            name=name,
            type="fixed",
            parent=parent,
            child=child,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            axis="",
            limits=""
        )
    
    def generate_revolute_joint(self, name, parent, child, xyz, rpy, axis, limits):
        """生成旋转关节"""
        x, y, z = xyz
        roll, pitch, yaw = rpy
        axis_x, axis_y, axis_z = axis
        lower, upper, effort, velocity = limits
        
        axis_str = self.axis_template.format(x=axis_x, y=axis_y, z=axis_z)
        limits_str = self.limits_template.format(
            lower=lower,
            upper=upper,
            effort=effort,
            velocity=velocity
        )
        
        return self.joint_template.format(
            name=name,
            type="revolute",
            parent=parent,
            child=child,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            axis=axis_str,
            limits=limits_str
        )
    
    def generate_continuous_joint(self, name, parent, child, xyz, rpy, axis):
        """生成连续旋转关节"""
        x, y, z = xyz
        roll, pitch, yaw = rpy
        axis_x, axis_y, axis_z = axis
        
        axis_str = self.axis_template.format(x=axis_x, y=axis_y, z=axis_z)
        
        return self.joint_template.format(
            name=name,
            type="continuous",
            parent=parent,
            child=child,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            axis=axis_str,
            limits=""
        )
    
    def generate_material(self, name, rgba):
        """生成材质"""
        r, g, b, a = rgba
        return self.material_template.format(name=name, r=r, g=g, b=b, a=a)
    
    def generate_diff_drive_plugin(self, namespace, left_joint, right_joint, wheel_separation, wheel_radius):
        """生成差速驱动插件"""
        content = f'''
      <ros>
        <namespace>{namespace}</namespace>
      </ros>
      <left_joint>{left_joint}</left_joint>
      <right_joint>{right_joint}</right_joint>
      <wheel_separation>{wheel_separation}</wheel_separation>
      <wheel_diameter>{wheel_radius * 2}</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>'''
        
        return self.gazebo_plugin_template.format(
            name="diff_drive",
            filename="libgazebo_ros_diff_drive.so",
            content=content
        )
    
    def generate_lidar_sensor(self, link_name, namespace):
        """生成激光雷达传感器"""
        properties = '''
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>'''
        
        plugin_content = f'''
        <ros>
          <namespace>{namespace}</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>{link_name}</frame_name>'''
        
        return self.sensor_template.format(
            link=link_name,
            name="lidar",
            type="ray",
            properties=properties,
            plugin_name="lidar_controller",
            plugin_filename="libgazebo_ros_ray_sensor.so",
            plugin_content=plugin_content
        )
    
    def generate_camera_sensor(self, link_name, namespace):
        """生成相机传感器"""
        properties = '''
      <pose>0 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <update_rate>30</update_rate>
      <visualize>true</visualize>'''
        
        plugin_content = f'''
        <ros>
          <namespace>{namespace}</namespace>
          <remapping>image_raw:=image</remapping>
          <remapping>camera_info:=camera_info</remapping>
        </ros>
        <frame_name>{link_name}</frame_name>'''
        
        return self.sensor_template.format(
            link=link_name,
            name="camera",
            type="camera",
            properties=properties,
            plugin_name="camera_controller",
            plugin_filename="libgazebo_ros_camera.so",
            plugin_content=plugin_content
        )
    
    def generate_differential_robot(self, robot_name, namespace, base_size, wheel_radius, wheel_width, wheel_separation):
        """生成差速驱动机器人模型"""
        urdf = self.template_header.format(robot_name=robot_name)
        
        # 添加材质
        urdf += self.generate_material("blue", [0, 0, 0.8, 1])
        urdf += self.generate_material("black", [0, 0, 0, 1])
        urdf += self.generate_material("white", [1, 1, 1, 1])
        
        # 添加基座链接
        base_length, base_width, base_height = base_size
        urdf += self.generate_box_link("base_link", [base_length, base_width, base_height], 5.0, "blue")
        
        # 添加轮子链接
        urdf += self.generate_cylinder_link("right_wheel", wheel_radius, wheel_width, 1.0, "black")
        urdf += self.generate_cylinder_link("left_wheel", wheel_radius, wheel_width, 1.0, "black")
        
        # 添加万向轮链接
        caster_radius = wheel_radius / 2
        urdf += self.generate_sphere_link("front_caster", caster_radius, 0.5, "white")
        
        # 添加传感器链接
        urdf += self.generate_cylinder_link("lidar_link", 0.05, 0.05, 0.1, "black")
        urdf += self.generate_box_link("camera_link", [0.05, 0.1, 0.05], 0.1, "black")
        
        # 添加轮子关节
        urdf += self.generate_continuous_joint(
            "right_wheel_joint", "base_link", "right_wheel", 
            [0, -wheel_separation/2, 0], [np.pi/2, 0, 0], [0, 0, 1]
        )
        
        urdf += self.generate_continuous_joint(
            "left_wheel_joint", "base_link", "left_wheel", 
            [0, wheel_separation/2, 0], [np.pi/2, 0, 0], [0, 0, 1]
        )
        
        # 添加万向轮关节
        urdf += self.generate_fixed_joint(
            "front_caster_joint", "base_link", "front_caster",
            [base_length/3, 0, -base_height/2], [0, 0, 0]
        )
        
        # 添加传感器关节
        urdf += self.generate_fixed_joint(
            "lidar_joint", "base_link", "lidar_link",
            [0, 0, base_height/2 + 0.05/2], [0, 0, 0]
        )
        
        urdf += self.generate_fixed_joint(
            "camera_joint", "base_link", "camera_link",
            [base_length/2 - 0.05/2, 0, base_height/2], [0, 0, 0]
        )
        
        # 添加差速驱动插件
        urdf += self.generate_diff_drive_plugin(
            namespace, "left_wheel_joint", "right_wheel_joint", wheel_separation, wheel_radius
        )
        
        # 添加激光雷达传感器
        urdf += self.generate_lidar_sensor("lidar_link", namespace)
        
        # 添加相机传感器
        urdf += self.generate_camera_sensor("camera_link", namespace)
        
        # 添加Gazebo颜色设置
        urdf += '''
  <!-- 设置材质颜色 -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>
  
  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>
  
  <gazebo reference="front_caster">
    <material>Gazebo/White</material>
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>
  
  <gazebo reference="lidar_link">
    <material>Gazebo/Black</material>
  </gazebo>
  
  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>
'''
        
        urdf += self.template_footer
        
        # 格式化XML
        dom = xml.dom.minidom.parseString(urdf)
        pretty_xml = dom.toprettyxml(indent='  ')
        
        return pretty_xml
    
    def save_model(self, model_xml, output_path):
        """保存模型到文件"""
        with open(output_path, 'w') as f:
            f.write(model_xml)
        print(f"模型已保存到: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='生成URDF机器人模型')
    parser.add_argument('--name', type=str, default='custom_robot', help='机器人名称')
    parser.add_argument('--namespace', type=str, default='/custom_robot', help='ROS命名空间')
    parser.add_argument('--base_length', type=float, default=0.5, help='基座长度')
    parser.add_argument('--base_width', type=float, default=0.3, help='基座宽度')
    parser.add_argument('--base_height', type=float, default=0.15, help='基座高度')
    parser.add_argument('--wheel_radius', type=float, default=0.1, help='轮子半径')
    parser.add_argument('--wheel_width', type=float, default=0.05, help='轮子宽度')
    parser.add_argument('--wheel_separation', type=float, default=0.4, help='轮子间距')
    parser.add_argument('--output', type=str, default='custom_robot.urdf.xacro', help='输出文件路径')
    
    args = parser.parse_args()
    
    generator = URDFModelGenerator()
    model_xml = generator.generate_differential_robot(
        args.name,
        args.namespace,
        [args.base_length, args.base_width, args.base_height],
        args.wheel_radius,
        args.wheel_width,
        args.wheel_separation
    )
    
    generator.save_model(model_xml, args.output)
    print(f"可以使用以下命令查看模型:\ncheck_urdf {args.output}")

if __name__ == '__main__':
    main()
