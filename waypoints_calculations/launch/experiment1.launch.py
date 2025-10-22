#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, LogInfo
import os

def generate_launch_description():
    """
    Experiment 1: Basic Path Tracking
    
    This launch file:
    1. Loads a predefined test path
    2. Starts the path tracking controller
    3. Starts the metrics collection node
    """
    
    # Path to test files
    waypoints_path = '/home/armdut/waypoints/test_straight.csv'
    
    return LaunchDescription([
        # Log experiment start
        LogInfo(msg=[
            '\n',
            '╔════════════════════════════════════════════════╗\n',
            '║  EXPERIMENT 1: BASIC PATH TRACKING             ║\n',
            '╠════════════════════════════════════════════════╣\n',
            '║  Testing: Ackermann kinematics & control      ║\n',
            '║  Path: Straight line (20m)                     ║\n',
            '║  Metrics: CTE, Heading Error, Speed            ║\n',
            '╚════════════════════════════════════════════════╝\n'
        ]),
        
        # Start waypoints loader
        Node(
            package='waypoints_niagara_loader',
            executable='waypoints_loader_node',
            name='waypoints_loader',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # Start path tracking controller
        Node(
            package='waypoints_calculations',
            executable='waypoints_calculations',
            name='path_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        # Start metrics collection (delayed to ensure other nodes are ready)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='📊 Starting metrics collection...'),
                Node(
                    package='waypoints_calculations',
                    executable='PathTrackingMetrics',
                    name='metrics_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True
                    }]
                )
            ]
        ),
        
        # Log instructions
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg=[
                    '\n',
                    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n',
                    '  EXPERIMENT RUNNING\n',
                    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n',
                    '  The car should now follow the test path.\n',
                    '  Metrics are being collected in real-time.\n',
                    '\n',
                    '  Press Ctrl+C to stop and view results.\n',
                    '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
                ])
            ]
        )
    ])
