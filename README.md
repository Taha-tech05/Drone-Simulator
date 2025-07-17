ModelData.txt file was used for triaining data for machine learning model to check if our pathfinding was correct and according to AI.


I built a smart drone simulation system that navigates through a complex 2D environment — using the A* (A-Star) AI algorithm to find the most efficient and safe path from start to goal. 📡

🗺️ The Environment:
The drone's terrain is a carefully designed grid with:

🟫 'C' – Common terrain (default cost)

🌪️ 'W' – Windy areas (passable but with higher cost due to instability)

🚫 '#' – Obstacles (completely impassable)

The drone must fly only through the inner, safe part of the terrain, intelligently avoiding edges and all hazardous zones.

🧠 Why A* Is AI
The A* algorithm is a classic Artificial Intelligence search technique that finds optimal paths using informed decisions. It combines:

g(n): Cost from the start to the current node

h(n): Estimated cost from the current node to the goal (we used Manhattan distance for h(n))

f(n) = g(n) + h(n) determines the most promising next move

Here’s how AI shows up:
✔️ The drone thinks ahead, estimating not just how far but how hard the path will be.
✔️ It adapts dynamically to higher-weight areas like 'W', avoiding them if safer routes exist.
✔️ It doesn't just find a path — it finds the best one based on risk, distance, and terrain type.

⚙️ Core Features I Implemented:

📍 Custom grid system with terrain encoded as a 2D char[][] array

🚧 Full A* logic: open/closed lists, cost tracking, and parent path recovery

🌪️ Windy zones with added movement cost, increasing the search complexity

📉 Visual and console-based path reconstruction from start to goal

🔍 Takeaways:
✅ Learned how to apply AI in real-time navigation scenarios
✅ Understood how to assign and handle variable path weights
✅ Designed around real-world considerations like wind, obstacles, and limited space
