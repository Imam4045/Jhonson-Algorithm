# 🗺️ Johnson's Algorithm: All-Pairs Shortest Path

This project is a console-based implementation of Johnson's Algorithm written in C++. It finds the shortest paths between every pair of vertices in a weighted directed graph, including graphs with negative edge weights, as long as there are no negative-weight cycles.

---

## 📚 Table of Contents

- [What This Program Can Do](#-what-this-program-can-do)
- [How The Algorithm Works](#-how-the-algorithm-works)
- [Algorithm Flowchart](#-algorithm-flowchart)
- [How To Run The Project](#%EF%B8%8F-how-to-run-the-project)
- [Input Format](#%EF%B8%8F-input-format)
- [Sample Output](#-sample-output)
- [Time Complexity](#%EF%B8%8F-time-complexity)
- [Important Notes](#-important-notes)
- [Requirements](#%EF%B8%8F-requirements)
- [Contributing](#-contributing)
- [License](#-license)
- [Contact](#-contact)

---

## 🚀 What This Program Can Do

- Detect negative-weight cycles using the Bellman-Ford algorithm
- Re-weight all edges to eliminate negative weights while preserving shortest path properties
- Compute shortest paths between every pair of vertices using Dijkstra's algorithm
- Handle graphs with negative edge weights (but no negative cycles)
- Report "No path" for any pair of vertices with no valid route
- Immediately terminate and report if a negative-weight cycle is detected

---

## 🧠 How The Algorithm Works

Johnson's Algorithm finds the shortest path between **every pair of vertices** in a weighted directed graph. It combines Bellman-Ford and Dijkstra. Bellman-Ford runs **once** to handle negative weights, then the faster Dijkstra handles the rest.

- **Step 1:** Add a temporary source node with zero-weight edges to all vertices
- **Step 2:** Run Bellman-Ford to compute a potential value `h[v]` for each vertex and detect any negative-weight cycles
- **Step 3:** Re-weight every edge using `new weight = old weight + h[u] − h[v]` so all weights become non-negative
- **Step 4:** Run Dijkstra from every vertex on the re-weighted graph
- **Step 5:** Recover true distances using `true distance = dijkstra result − h[u] + h[v]`

---

## 📊 Algorithm Flowchart

A detailed visual flowchart of the algorithm is included in this repository.

📄 [View Jhonson's Algorithm Flowchart (PDF)](./Jhonson's%20Algorithm%20Flowchart.pdf)

---

## ⚙️ How To Run The Project

### Step 1: Install Required Tools

- Install [Visual Studio Code](https://code.visualstudio.com/)
- Install the **C/C++ Extension** by Microsoft from the VS Code Extensions panel
- Install [MinGW-w64](https://www.mingw-w64.org/) and add `C:\mingw64\bin` to your system PATH

### Step 2: Open The Project

Open VS Code, go to **File → Open Folder** and select the folder containing `Jhonson's Algorithm.cpp`.

### Step 3: Configure VS Code

Make sure your `.vscode/tasks.json` contains the following:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "type": "shell",
            "label": "C/C++: g++.exe build active file",
            "command": "C:\\mingw64\\bin\\g++.exe",
            "args": [
                "-fdiagnostics-color=always",
                "-g",
                "${fileBasename}",
                "-o",
                "${fileBasenameNoExtension}.exe",
                "-static-libgcc",
                "-static-libstdc++"
            ],
            "options": {
                "cwd": "${fileDirname}"
            },
            "problemMatcher": ["$gcc"],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        }
    ]
}
```

### Step 4: Compile and Run

Open `Jhonson's Algorithm.cpp` in the editor, then press **Ctrl+Alt+N** to compile and run. A terminal will open at the bottom of VS Code where you can enter your input.

---

## 🖥️ Input Format

Once the program starts, it will ask:

    Vertices:
    Edges:

After that, enter each directed edge on a new line in this format:

    <source> <destination> <weight>

**Example input:**

    Vertices: 5
    Edges: 9
    0 1 3
    0 2 8
    0 3 2
    0 4 -4
    1 4 7
    1 3 1
    2 1 4
    3 2 -5
    4 3 6

- Vertices are **0-indexed**
- Edges are **directed** (one-way)
- Negative weights are **allowed**, negative-weight cycles are **not**

---

## 🧪 Sample Output

### When there is no negative cycle

    Shortest distance from 0 to 0 is 0
    Shortest distance from 0 to 1 is 1
    Shortest distance from 0 to 2 is -3
    Shortest distance from 0 to 3 is 2
    Shortest distance from 0 to 4 is -4
    No path from 1 to 0
    Shortest distance from 1 to 2 is -4
    Shortest distance from 1 to 3 is 1
    Shortest distance from 1 to 4 is 7
    ...

### When there is a negative cycle

    Negative weight cycle detected

---

## ⏱️ Time Complexity

| Component | Complexity |
| --- | --- |
| Bellman-Ford | O(V · E) |
| Edge Re-weighting | O(E) |
| Dijkstra (run V times) | O(V(V + E) log V) |
| **Total** | **O(V² log V + V · E)** |

---

## 📝 Important Notes

- Vertices must be **0-indexed** starting from `0`.
- The algorithm works only on **directed** graphs.
- Negative edge weights are **allowed** but negative-weight **cycles** will cause the program to terminate early.
- The self-distance for every vertex (from any node to itself) is always **0**.
- Always enter the number of vertices and edges correctly before inputting edges.

---

## 🛠️ Requirements

- MinGW-w64 with `C:\mingw64\bin` added to system PATH
- Windows OS
- Visual Studio Code with the C/C++ extension

---

## 🤝 Contributing

If you have any suggestions or want to improve the project, feel free to fork it, make your changes and submit a pull request.

---

## 🔒 License

This project is licensed under the [MIT License](./LICENSE).

---

## 📬 Contact

If you have any questions or concerns, please don't hesitate to contact me via email at imam220826@gmail.com
