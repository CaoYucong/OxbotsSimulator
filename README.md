# Oxbots Software



# ⚠️ IMPORTANT — Pause & Reset Before Editing

Webots **automatically starts running the simulation as soon as you open a world**.  

If you edit the scene while the simulation is running, **Webots WILL NOT save your changes**, even if you press `Cmd+Shift+S`.



To avoid losing work:

### ✔ Step 1 — Immediately press **Pause** (⏸)  

### ✔ Step 2 — Then press the **Reset Simulation** button ⏮️:

### ✔ Step 3 — Only after Pause + Reset, you may safely edit the file. 

## ❌ If you forget:

- Webots treats all edits as temporary runtime changes  

- **Closing or resetting the world discards them**

- This comes from many horrible stories 😭



## 1. Simulator

[Link to download.](https://cyberbotics.com)

## 2. **Repository Structure**

```
OxbotsSimulator
│
├── controllers               # Not investigated yet
│
├── protos                  
│   ├── AprilTag.proto        # One Single Apriltag
│   ├── AprilTagWall.proto    # Fit 24 Apriltags on the wall
│   ├── Pingball.proto        # Ping-Pong Ball
│   ├── SteelBall.proto       # Steel Ball
│   ├── UnibotsArena.proto    # Base Arena
│   ├── UnibotsBalls.proto    # 40 Balls
│   └── UnibotsBallsRandom.proto # 40 Balls, randomly positioned, see \tools.
│
├── textures
│   └── tag36h11              # AprilTag Family tag36h11, numbered 0-23
│       ├── tag36h11-0.png
│       ├── tag36h11-1.png
│       ...
│       ├── tag36h11-21.png
│       ├── tag36h11-22.png
│       └── tag36h11-23.png   # 24 AprilTags in total
│ 
├── tools
│   └── generate_random_balls_proto.py  # Generate random ball confituration
│                                       # with auto back-up previous ball confituration
│ 
├── worlds/                   # Simulation entry points
│   ├── OxBots_Arena.wbt      # Arena developing file, no use
│   └── Run.wbt					      # Run this
│
├── README.md
├── Rulebook Unibots 2026 V1.1 - 02 Oct 2025.pdf
```

All other files are irrelevant to the project and are demos from the software, I kept them just for development reference.

## 3. Running the Simulation

### ✔ Step 1 — Launch Webots  

Open **Webots.app**

### ✔ Step 2 — Open the arena world  

In Webots:

```
file -> Open World
```

Select:

```
worlds/Run.wbt
```

### ✔ You should see:

```
1. The full UniBots arena
	•	A 2 m × 2 m white square floor
	•	Four colored walls:
	•	Yellow (North)
	•	Orange (East)
	•	Green (West)
	•	Purple (South)
	•	Four scoring net structures mounted outside the walls

2. All 40 balls placed in the arena
	•	16 yellow ping-pong balls 
	•	24 steel balls 
	•	Positioned according to the PROTO configuration 
```

