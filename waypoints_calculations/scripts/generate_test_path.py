#!/usr/bin/env python3

import csv
import numpy as np
import os
import sys

def generate_straight_path(length=20, spacing=0.5):
    """Generate a straight path along X-axis"""
    waypoints = []
    for x in np.arange(0, length, spacing):
        # [x, y, z, yaw, velocity, change_flag]
        waypoints.append([x, 0.0, 0.0, 0.0, 2.0, 0])  
    return waypoints

def generate_curved_path(radius=10, angle_degrees=90, spacing=0.5):
    """Generate a curved path (arc)"""
    waypoints = []
    angle_rad = np.radians(angle_degrees)
    arc_length = radius * angle_rad
    num_points = int(arc_length / spacing)
    
    for i in range(num_points):
        theta = (i / num_points) * angle_rad
        x = radius * np.sin(theta)
        y = radius * (1 - np.cos(theta))
        # Calculate yaw (tangent direction)
        yaw = theta
        # [x, y, z, yaw, velocity, change_flag]
        waypoints.append([x, y, 0.0, yaw, 2.0, 0])
    return waypoints

def generate_straight_only(output_file):
    """Generate simple straight path for initial testing"""
    waypoints = []
    
    # Straight segment: 20 meters
    straight = generate_straight_path(length=20, spacing=0.5)
    waypoints.extend(straight)
    
    # Save to CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])  # Header
        writer.writerows(waypoints)
    
    print(f"✅ Generated {len(waypoints)} waypoints (Straight Path)")
    print(f"📁 Saved to: {output_file}")

def generate_curved_only(output_file):
    """Generate curved path only"""
    waypoints = []
    
    # Curved segment: 90-degree turn
    curve = generate_curved_path(radius=8, angle_degrees=90, spacing=0.5)
    waypoints.extend(curve)
    
    # Save to CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])  # Header
        writer.writerows(waypoints)
    
    print(f"✅ Generated {len(waypoints)} waypoints (Curved Path)")
    print(f"📁 Saved to: {output_file}")

def generate_combined_path(output_file):
    """Generate straight + curve + straight path"""
    waypoints = []
    
    # Straight segment 1: 15 meters (yaw = 0)
    for x in np.arange(0, 15, 0.5):
        waypoints.append([x, 0.0, 0.0, 0.0, 2.0, 0])
    
    # Curved segment: 90-degree turn
    last_x, last_y = waypoints[-1][0], waypoints[-1][1]
    radius = 8
    for i in range(25):  # ~12.5m arc
        theta = (i / 25.0) * np.pi/2
        x = last_x + radius * np.sin(theta)
        y = last_y + radius * (1 - np.cos(theta))
        yaw = theta  # Tangent direction
        waypoints.append([x, y, 0.0, yaw, 2.0, 0])
    
    # Straight segment 2: 10 meters (yaw = 90 degrees)
    last_x, last_y = waypoints[-1][0], waypoints[-1][1]
    for i in range(20):  # 10m at 0.5m spacing
        x = last_x
        y = last_y + i * 0.5
        waypoints.append([x, y, 0.0, np.pi/2, 2.0, 0])
    
    # Save to CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])  # Header
        writer.writerows(waypoints)
    
    print(f"✅ Generated {len(waypoints)} waypoints (Combined Path)")
    print(f"📁 Saved to: {output_file}")

def generate_figure_eight(output_file):
    """Generate figure-8 path for advanced testing"""
    waypoints = []
    radius = 6
    spacing = 0.5
    
    # First loop
    for angle in np.arange(0, 360, spacing / radius * 180 / np.pi):
        theta = np.radians(angle)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        yaw = theta + np.pi/2  # Tangent direction
        waypoints.append([x, y, 0.0, yaw, 2.0, 0])
    
    # Second loop (offset)
    for angle in np.arange(0, 360, spacing / radius * 180 / np.pi):
        theta = np.radians(angle)
        x = radius * np.cos(theta) + 2 * radius
        y = radius * np.sin(theta)
        yaw = theta + np.pi/2  # Tangent direction
        waypoints.append([x, y, 0.0, yaw, 2.0, 0])
    
    # Save to CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])  # Header
        writer.writerows(waypoints)
    
    print(f"✅ Generated {len(waypoints)} waypoints (Figure-8 Path)")
    print(f"📁 Saved to: {output_file}")

if __name__ == "__main__":
    # Create waypoints directory if it doesn't exist
    waypoints_dir = "/home/armdut/waypoints"
    os.makedirs(waypoints_dir, exist_ok=True)
    
    # Generate different test paths
    print("\n" + "="*50)
    print("Generating Test Paths for Experiment 1")
    print("="*50 + "\n")
    
    # Path 1: Straight line (simplest)
    generate_straight_only(os.path.join(waypoints_dir, "test_straight.csv"))
    print()
    
    # Path 2: Curved path only
    generate_curved_only(os.path.join(waypoints_dir, "test_curve.csv"))
    print()
    
    # Path 3: Combined (straight + curve + straight)
    generate_combined_path(os.path.join(waypoints_dir, "test_combined.csv"))
    print()
    
    # Path 4: Figure-8 (advanced)
    generate_figure_eight(os.path.join(waypoints_dir, "test_figure8.csv"))
    print()
    
    print("="*50)
    print("✅ All test paths generated successfully!")
    print("="*50 + "\n")
