## Documentation 

This is a simple C implementation of sensor fusion using a **Kalman Filter** to integrate data from multiple sensors (LiDAR, GPS, and IMU). This helps improve accuracy in positioning and environmental awareness.

Autonomous drones rely on several AI algorithms to navigate, make decisions, and optimize performance e.g.

1. **Computer Vision & Object Recognition** – Drones use AI-powered vision systems to detect, track, and classify objects in their surroundings, enabling obstacle avoidance and target identification.
2. **Path Planning & Navigation** – AI algorithms help drones calculate optimal flight paths, avoiding obstacles and adjusting routes dynamically.
3. **Sensor Fusion** – Combining data from multiple sensors (LiDAR, GPS, IMU) using AI improves accuracy in positioning and environmental awareness.
4. **Reinforcement Learning** – Drones learn from experience, improving flight efficiency and decision-making over time.
5. **Swarm Intelligence** – AI enables multiple drones to coordinate and work together efficiently, useful in search-and-rescue or surveillance operations.
6. **Predictive Maintenance** – AI analyzes drone performance data to predict failures and schedule maintenance proactively.
### **How it Works**
1. **Sensors Collect Data** – GPS, LiDAR, and IMU provide position estimates.
2. **Kalman Filter Applies Corrections** – It improves accuracy by weighing each sensor's reliability.
3. **Final Position Estimate** – The result is a refined and more precise position estimate.
