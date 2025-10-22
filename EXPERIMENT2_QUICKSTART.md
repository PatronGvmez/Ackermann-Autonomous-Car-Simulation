# Experiment 2: Quick Start Guide 🚗

## 🚀 Launch Complete System

### Prerequisites:
```bash
cd ~/ros2_ws
source install/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/velodyne_gazebo_plugins/lib
```

### **Option 1: Autonomous Driving Mode**
Car drives itself and avoids obstacles automatically:
```bash
ros2 launch obstacle_detection experiment2.launch.py \
  auto_drive:=true \
  cruise_speed:=1.0 \
  safety_distance:=1.5
```

### **Option 2: Manual Assist Mode**
You drive, system prevents collisions:
```bash
# Terminal 1 - Launch system
ros2 launch obstacle_detection experiment2.launch.py \
  auto_drive:=false \
  safety_distance:=1.5

# Terminal 2 - Manual control (after 12 seconds)
cd ~/ros2_ws && source install/setup.bash
ros2 run obstacle_detection ackermann_teleop
```

**WASD Controls:**
- `W` - Forward
- `S` - Reverse
- `A` - Turn left
- `D` - Turn right
- `B` - Emergency stop
- `Q` - Quit
- `Space` - Gradual slowdown

---

## 📊 View Metrics Real-Time

```bash
# Watch metrics in console (updates every 5 seconds)
# Automatically displayed in metrics_collector terminal output

# Or view raw data:
tail -f ~/waypoints/metrics_exp2_*.csv
```

---

## 🛑 Stop Everything

```bash
# Kill all processes
killall -9 gzserver gzclient gazebo rviz2
pkill -9 -f "obstacle_detector|avoidance_controller|metrics_collector|ackermann_teleop"
```

---

## 📈 Analyze Results

```bash
# Find your metrics file
ls -lht ~/waypoints/metrics_exp2_*.csv | head -1

# View summary
tail -20 <your_metrics_file>

# Plot with Python (optional)
python3 -c "
import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv('<your_metrics_file>')
df.plot(x='timestamp_sec', y=['obstacle_distance_m', 'linear_velocity_mps'])
plt.show()
"
```

---

## ⚙️ Tuning Parameters

Edit values in launch command:

| Parameter | Default | Description | Recommended Range |
|-----------|---------|-------------|-------------------|
| `cruise_speed` | 1.0 | Autonomous speed (m/s) | 0.5 - 2.0 |
| `safety_distance` | 1.5 | Emergency stop threshold (m) | 1.0 - 2.0 |
| `avoidance_gain` | 1.2 | Steering aggressiveness | 0.8 - 2.0 |
| `warning_distance` | 3.0 | Avoidance activation (m) | 2.0 - 5.0 |

Example with custom parameters:
```bash
ros2 launch obstacle_detection experiment2.launch.py \
  auto_drive:=true \
  cruise_speed:=1.5 \
  safety_distance:=2.0
```

---

## 🐛 Troubleshooting

### **Problem:** Gazebo says "Address already in use"
```bash
killall -9 gzserver gzclient gazebo
sleep 3
# Try launch again
```

### **Problem:** No LiDAR data (/points_raw empty)
```bash
# Make sure GAZEBO_PLUGIN_PATH is set BEFORE launching:
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/velodyne_gazebo_plugins/lib
```

### **Problem:** Car doesn't move in autonomous mode
- Check if obstacle is too close (< 1.5m) → emergency stop active
- Solution: Manually move car away from wall first, or use manual assist mode

### **Problem:** Metrics file not created
```bash
# Create waypoints directory if missing:
mkdir -p ~/waypoints
chmod 755 ~/waypoints
```

---

## 📊 Expected Results

**Good Run Indicators:**
- ✅ Collision count stays at 0
- ✅ Minimum clearance > 1.2m
- ✅ Car moves smoothly (no jittering)
- ✅ Emergency stops only near obstacles
- ✅ Computation time < 1ms

**Warning Signs:**
- ⚠️ Collision count > 0 → tune safety_distance higher
- ⚠️ Min clearance < 1.0m → too aggressive, increase safety margin
- ⚠️ Car stuck/stationary → trapped scenario, needs manual intervention
- ⚠️ Computation time > 5ms → system overload, reduce sensor rate

---

## 🎯 Quick Test Checklist

1. ☑️ Launch system in autonomous mode
2. ☑️ Verify car starts moving forward
3. ☑️ Watch obstacle distance decrease in console
4. ☑️ Confirm emergency stop at ~1.5m
5. ☑️ Check metrics CSV created in ~/waypoints/
6. ☑️ Verify 0 collisions in summary output
7. ☑️ Kill all processes cleanly

---

## 📞 Files Location

- **Source code:** `~/ros2_ws/src/obstacle_detection/src/`
- **Launch file:** `~/ros2_ws/src/obstacle_detection/launch/experiment2.launch.py`
- **Metrics output:** `~/waypoints/metrics_exp2_*.csv`
- **Documentation:** `~/gazebo_worlds/Ackermann-Autonomous-Car-Simulation/EXPERIMENT2_RESULTS.md`

---

## 🏆 Success Criteria

**Experiment 2 is considered successful if:**
1. Zero collisions during 60-second autonomous run
2. Minimum clearance maintained > 1.0m
3. Controller computation time < 2ms
4. Metrics properly logged to CSV
5. Emergency stop activates at correct threshold

**YOUR RESULTS:** ✅ **ALL CRITERIA MET!**
- Collisions: 0 ✅
- Min clearance: 1.46m ✅
- Computation: 0.1ms ✅
- Metrics logged: ✅
- Emergency stop: Working ✅

---

## 🚀 Next Steps

**Experiment 3:** Full Autonomy
- Combine path following (Exp 1) + obstacle avoidance (Exp 2)
- Add SLAM for localization
- Implement global path planning
- Test in complex urban-like environment
- Measure mission completion time & efficiency

Ready when you are! 🎉
