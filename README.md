# 3D Pathfinding Visualizer

An interactive web application to visualize and compare pathfinding algorithms in a 3D space. 
This project allows you to create custom obstacles, set start and goal points, and execute different algorithms to find the optimal paths.

![Application Screenshot](placeholder_image.png)

## 📋 Features
- **Interactive 3D Visualization:** Navigate the space, rotate the camera, and examine paths and obstacles in full 3D.
- **Customizable Grid:** Modify the world size (e.g., 10x10x10) and dynamically place blocks as obstacles.
- **Multiple Algorithms:**
  - **A***: The standard and balanced pathfinding algorithm.
  - **Theta***: An optimized A* variant with Line of Sight for more natural trajectories.
  - **D* Lite**: A dynamic variant that adapts to environmental changes, ideal for variable settings.
  - **JPS (Jump Point Search)**: A highly optimized algorithm for uniform grids.
- **Comparison Tool:** Visually compare the generated paths and check execution times and step counts in a convenient table.
- **Premium Theme:** A dark "cyber/neon" interface, data-focused and easy to read.

## 🏗 Project Structure

The project has been refactored to be modular and maintainable:

```text
3D-Pathfinding/
├── README.md                 # This file
├── Dockerfile                # Docker configuration
├── docker-compose.yaml       # Container orchestration
├── nginx.conf                # Nginx server configuration
├── pages/
│   └── index.html            # Main HTML entry point of the app
├── css/
│   └── style.css             # Isolated stylesheets
└── js/
    ├── app.js                # Main UI logic and Three.js setup
    ├── core/
    │   ├── utils.js          # Shared data structures (MinHeap, distance utilities)
    │   └── octree.js         # Octree class for 3D space indexing and collision management
    └── algorithms/
        ├── astar.js          # A* Implementation
        ├── thetastar.js      # Theta* Implementation
        ├── dstar.js          # D* Lite Implementation
        └── jps.js            # Jump Point Search (3D) Implementation
```

## 🚀 How to Run

### Using Docker (Recommended)

1. Make sure you have [Docker](https://www.docker.com/) and `docker-compose` installed on your system.
2. Open the terminal in the root directory of the project (`3D-Pathfinding`).
3. Run the following command:
   ```bash
   docker-compose up --build -d
   ```
4. Open your browser and go to: **http://localhost:80** (or the port configured in your docker-compose file).
5. To stop the project:
   ```bash
   docker-compose down
   ```

### Without Docker (Local Development)

Since the project uses ES modules and external files, you need a local web server to bypass CORS restrictions on `file://` protocols.
You can use VSCode extensions (e.g., **Live Server**) or a quick terminal server:

**With Python:**
```bash
python3 -m http.server 8000
# Then open http://localhost:8000/pages/index.html
```

**With Node.js (http-server):**
```bash
npx http-server -p 8000
# Then open http://localhost:8000/pages/index.html
```

## 📸 Images / Screenshots

*You can insert your application screenshots below:*

1. **Main Interface:**
   ![Main UI](placeholder_ui.png)
   
2. **Algorithm Comparison:**
   ![Comparison](placeholder_comparison.png)

3. **Generated Path Detail:**
   ![Path Detail](placeholder_path.png)

## 🛠 Technologies Used
- **HTML5 / CSS3** (No CSS frameworks)
- **Vanilla JavaScript** (ES6+)
- **[Three.js](https://threejs.org/)** (R128 for WebGL rendering)
- **Nginx & Docker** (For deployment)
