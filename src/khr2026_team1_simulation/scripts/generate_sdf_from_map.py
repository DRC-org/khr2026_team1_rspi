#!/usr/bin/env python3
import yaml
import cv2
import numpy as np
import os
import argparse

def generate_sdf(map_yaml_path, output_sdf_path):
    map_dir = os.path.dirname(map_yaml_path)
    
    with open(map_yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    
    pgm_path = os.path.join(map_dir, map_info['image'])
    resolution = map_info['resolution']
    origin = map_info['origin']
    occupied_thresh = map_info.get('occupied_thresh', 0.65)
    
    # Read the image in grayscale
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Failed to load image: {pgm_path}")
        return False

    # Convert to binary map (obstacle = 255, free = 0)
    # The occupied_thresh is usually between 0.0 and 1.0, representing probability
    # In PGM, 0 is black (obstacle), 255 is white (free). We flip it so obstacles are white.
    thresh_val = int((1.0 - occupied_thresh) * 255)
    _, thresh = cv2.threshold(img, thresh_val, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    # Using RETR_LIST or RETR_TREE to get inner walls if surrounded by mapped free space
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    with open(output_sdf_path, "w") as f:
        f.write('''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="khr2026_field_generated">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

''')
        height = img.shape[0]

        for i, contour in enumerate(contours):
            # To make hollow walls, we draw thin boxes between consecutive points of the contour
            # Simplify contour to reduce number of walls
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            for j in range(len(approx)):
                pt1 = approx[j][0]
                pt2 = approx[(j+1)%len(approx)][0]
                
                x1_px, y1_px = pt1
                x2_px, y2_px = pt2
                
                # Transform to world coords
                x1_m = x1_px * resolution + origin[0]
                y1_m = (height - y1_px) * resolution + origin[1]
                x2_m = x2_px * resolution + origin[0]
                y2_m = (height - y2_px) * resolution + origin[1]
                
                # Calculate Wall length, center, and yaw
                dx = x2_m - x1_m
                dy = y2_m - y1_m
                length = np.sqrt(dx**2 + dy**2)
                
                if length < 0.05: continue # Skip tiny segments
                
                cx = x1_m + dx/2.0
                cy = y1_m + dy/2.0
                yaw = np.arctan2(dy, dx)
                
                thickness = 0.05
                size_z = 0.5
                
                f.write(f'''
    <!-- Wall Segment {i}_{j} -->
    <model name="auto_wall_{i}_{j}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {size_z/2:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{length:.3f} {thickness:.3f} {size_z:.3f}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{length:.3f} {thickness:.3f} {size_z:.3f}</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
''')
        f.write('''
    <!-- 3D障害物 / ターゲット (3D Obstacles / Targets) -->
    <model name="supply_y">
      <static>true</static>
      <pose>0.54 0.6 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.4 0.4 1.0</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.4 0.4 1.0</size></box></geometry><material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material></visual>
      </link>
    </model>
    <model name="supply_r">
      <static>true</static>
      <pose>2.8 6.2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>0.4 0.4 1.0</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>0.4 0.4 1.0</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>
      </link>
    </model>

    <model name="jintori_1">
      <static>true</static>
      <pose>0.35 2.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
    <model name="jintori_2">
      <static>true</static>
      <pose>0.35 3.2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
    <model name="jintori_3">
      <static>true</static>
      <pose>0.35 4.4 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
    <model name="honmaru">
      <static>true</static>
      <pose>0.25 1.40 0.6 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>0.05</radius><length>1.2</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>0.05</radius><length>1.2</length></cylinder></geometry><material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material></visual>
      </link>
    </model>
  </world>
</sdf>
''')
    print(f"Generated hollow walls from {pgm_path} into {output_sdf_path}")
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', type=str, required=True, help="Path to field.yaml")
    parser.add_argument('--out', type=str, required=True, help="Path to output SDF file")
    args = parser.parse_args()
    
    generate_sdf(args.yaml, args.out)
