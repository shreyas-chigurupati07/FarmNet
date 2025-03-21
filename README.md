# FarmNet - Autonomous Navigation of Quadrotor Drones Using A* Algorithm

## Project Overview
This project explores the autonomous navigation of a Bitcraze Crazyflie-modeled quadrotor drone in a Webots-simulated environment. The drone autonomously traverses a complex virtual landscape, avoiding static obstacles to reach predefined goals.

## Key Features
- **Pathfinding Algorithms**: Implemented and compared various algorithms, with the A* algorithm demonstrating superior efficiency and path optimization.
- **Simulation Environment**: Utilized Cyberbotics' Webots to create a realistic 3D simulation of a cattle field, including dynamic object placement and collision detection.
- **PID Control**: Fine-tuned PID controllers ensured stable flight and precise altitude maintenance, crucial for the drone's effective navigation and task execution.

## Results
The drone successfully navigated through the virtual environment, maintaining a consistent altitude and adapting its path in real-time to avoid obstacles. The A* algorithm's heuristic approach enabled efficient and smooth trajectory planning, outperforming other tested algorithms.

## Conclusion
The project underscores the effectiveness of the A* algorithm in real-time autonomous drone navigation, highlighting the critical role of precise control systems in achieving robust and reliable performance in complex environments.

## Future Work
Future enhancements include optimizing the A* algorithm for larger search areas, exploring adaptive control systems for improved stability at higher speeds, and expanding the simulation to dynamic environments with moving obstacles.

## How to Use
1. **Setup**: Clone the repository and set up the Webots simulation environment.
2. **Run Simulation**: Execute the provided script to launch the quadrotor simulation.
3. **Analyze Results**: Observe the drone's pathfinding in action and analyze the performance metrics.

## Contributors
- Shreyas Chigurupati
- Howard Ho
- Philip Ni

## References
- Detailed references to literature and utilized tools are included in the report.

