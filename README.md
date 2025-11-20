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
Simulator/
│
├── controllers/         # Not investigated yet
│
├── protos/              # Custom arena and ball models
│   ├── UnibotsArena.proto
│   ├── UnibotsBalls.proto
│
├── worlds/              # Simulation entry points
│   ├── OxBots_Arena.wbt # Arena developing file, no use
│   └── Run.wbt					 # Run this
│
└── README.md
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

