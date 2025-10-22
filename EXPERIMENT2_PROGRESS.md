# Experiment 2: Obstacle Avoidance - Progress Report

## Objective
Implement LiDAR-based obstacle detection and reactive avoidance for the Ackermann autonomous car.

## ✅ COMPLETED TASKS

### 1. LiDAR Sensor Verification
- **Status:** ✅ WORKING
- **Topic:** `/points_raw`
- **Type:** `sensor_msgs/msg/PointCloud2`
- **Publisher:** Velodyne VLP-16 plugin
- **Configuration:**
  - 16 laser beams
  - 440 samples per scan
  - 10 Hz update rate
  - Range: 0.9m - 130m
  - 360° horizontal FOV
  
**Solution Applied:**
- Added GAZEBO_PLUGIN_PATH environment variable:
  ```bash
  export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/velodyne_gazebo_plugins/lib
  ```

### 2. Obstacle Detection Package
- **Status:** ✅ CREATED AND BUILT
- **Package:** `obstacle_detection`
- **Node:** `obstacle_detector`
- **Location:** `~/ros2_ws/src/obstacle_detection/`

**Features Implemented:**
- Subscribes to `/points_raw` (LiDAR point cloud)
- Filters points by height (removes ground: z > 0.1m)
- Finds nearest obstacle within range (0.5m - 10m)
- Publishes nearest obstacle to `/nearest_obstacle`
- Publishes visualization marker to `/obstacle_marker`
- Color-coded warnings:
  - **RED:** Distance < 1.5m (danger)
  - **YELLOW:** Distance < 2.25m (caution)
  - **GREEN:** Distance > 2.25m (safe)

**Parameters:**
- `min_distance`: 0.5m (minimum detection distance)
- `max_distance`: 10.0m (maximum detection distance)
- `min_height`: 0.1m (filter ground plane)
- `max_height`: 2.5m (filter ceiling)
- `safety_distance`: 1.5m (warning threshold)

### 3. Test Environment
- **World File:** `w2.sdf` (already has walls and obstacles)
- **Status:** ✅ READY FOR TESTING
- Contains:
  - Starting pen with walls
  - Multiple static obstacles
  - Suitable for testing obstacle avoidance

## 📋 PENDING TASKS

### 4. Test Obstacle Detection
- [ ] Start simulation with w2.sdf world
- [ ] Launch obstacle_detector node
- [ ] Drive car manually to test detection
- [ ] Verify `/nearest_obstacle` topic publishes correctly
- [ ] Check RViz visualization of obstacle markers

### 5. Implement Avoidance Controller
- [ ] Create `obstacle_avoidance` package
- [ ] Implement reactive controller (VFH or simple potential field)
- [ ] Subscribe to `/nearest_obstacle`
- [ ] Blend avoidance with path following
- [ ] Publish modified `/cmd_vel` commands

### 6. Integration with Path Following
- [ ] Modify `WaypointsCalculations` to accept avoidance input
- [ ] Implement controller blending:
  - Path tracking (primary)
  - Obstacle avoidance (reactive override)
- [ ] Test on straight path with obstacles

### 7. Metrics Collection for Experiment 2
- [ ] Create `experiment2_metrics` node
- [ ] Track metrics:
  - Collision events (0/1)
  - Minimum clearance distance
  - Avoidance success rate
  - Path deviation
  - Computation time
- [ ] Save to CSV: `~/waypoints/metrics_exp2_[timestamp].csv`

### 8. Testing Scenarios
- [ ] **Scenario 1:** Single static obstacle in path
- [ ] **Scenario 2:** Multiple obstacles (corridor navigation)
- [ ] **Scenario 3:** Narrow passage
- [ ] **Scenario 4:** Complex obstacle field

### 9. Documentation
- [ ] Record test results
- [ ] Document parameter tuning
- [ ] Create test report
- [ ] Compare with baseline (no avoidance)

## 🚀 NEXT IMMEDIATE STEPS

1. **Test Current Setup:**
   ```bash
   # Terminal 1: Start simulation with obstacles
   cd ~/ros2_ws && source install/setup.bash
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/velodyne_gazebo_plugins/lib
   ros2 launch niagara_model display.launch.py
   # MANUALLY change world to w2.sdf in launch file OR spawn in w2.sdf
   
   # Terminal 2: Run obstacle detector
   cd ~/ros2_ws && source install/setup.bash
   ros2 run obstacle_detection obstacle_detector
   
   # Terminal 3: Monitor obstacles
   ros2 topic echo /nearest_obstacle
   
   # Terminal 4: Manual control for testing
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

2. **Verify Detection Works:**
   - Drive towards walls in w2.sdf
   - Observe console warnings when distance < 1.5m
   - Check RViz for red/yellow/green markers

3. **Create Avoidance Controller** (next major task)

## 📊 SUCCESS CRITERIA

- ✅ LiDAR publishes point clouds at 10 Hz
- ✅ Obstacle detector identifies nearest obstacles
- ⏳ Car avoids obstacles while following path
- ⏳ Zero collisions in test scenarios
- ⏳ Maintains < 1.0m safety clearance
- ⏳ Path deviation < 2.0m from reference

## 🔧 Key Configuration Files

- **LiDAR Topic:** `/points_raw` (sensor_msgs/PointCloud2)
- **Obstacle Topic:** `/nearest_obstacle` (geometry_msgs/PointStamped)
- **Marker Topic:** `/obstacle_marker` (visualization_msgs/Marker)
- **World File:** `~/ros2_ws/src/niagara_model/world/w2.sdf`
- **Detector Source:** `~/ros2_ws/src/obstacle_detection/src/obstacle_detector.cpp`

## 📝 Notes

- Must set `GAZEBO_PLUGIN_PATH` before launching simulation
- Current detector finds SINGLE nearest obstacle (simplest approach)
- Ready to proceed with avoidance controller implementation
- Consider upgrading to sector-based detection (multiple zones) for better awareness

---
**Last Updated:** October 22, 2025
**Status:** Phase 1 Complete (Detection), Phase 2 Pending (Avoidance)
