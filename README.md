<p align="center">
  <img src="https://img.shields.io/badge/Python-3.8+-blue?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/Status-Active-success?style=for-the-badge" alt="Status">
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge" alt="License">
</p>

# 🤖 Smart Logistics Simulation (SLS_ANSI)

> **Multi-Agent Autonomous Robot Simulation** สำหรับจำลองการทำงานของระบบ Logistics อัตโนมัติในคลังสินค้า พร้อม AI-powered Deadlock Prediction และ Real-time ANSI Terminal Visualization

---

## ✨ Features

- 🚀 **Multi-Robot Coordination** - จัดการหุ่นยนต์หลายตัวพร้อมกันโดยไม่ชนกัน
- 🧠 **AI-Powered Deadlock Prediction** - ใช้ ML Model ทำนายการติดขัดล่วงหน้า
- 🗺️ **Smart A\* Pathfinding** - อัลกอริทึมหาเส้นทางที่ปรับปรุงพร้อม dynamic cost calculation
- 🔄 **Automatic Deadlock Resolution** - แก้ไข deadlock อัตโนมัติด้วย yield/retreat strategies
- 📊 **Real-time ANSI Visualization** - แสดงผลแบบ real-time บน terminal
- ⚙️ **Configurable Scenarios** - ปรับแต่งสถานการณ์ผ่าน JSON config

---

## 📸 Demo

```
╔════════════════════════════════════════════════════════════════════════════════════════════════════╗
║                                   🤖 SMART LOGISTICS SIMULATION 🤖                                   ║
╚════════════════════════════════════════════════════════════════════════════════════════════════════╝
 Step: 42    | Time: 00:21 | Moves: 156 | Pickups: 8 | Dropoffs: 5 | Deadlocks: 2

📋 LEGEND:  IDLE   TO_PICKUP   DELIVERING   HOME   EVAC/YIELD  P=Pickup D=Dropoff

📊 STATISTICS:
   Progress: 5/25 (20.0%) [████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░]
   Robots: IDLE=5 | TO_PICKUP=8 | DELIVERING=5 | HOME=2 | EVAC=0
```

---

## 🚀 Quick Start

### Prerequisites

```bash
# Python 3.8+
python --version

# Required packages
pip install loguru pandas joblib numpy
```

### Installation

```bash
# Clone หรือ copy โปรเจกต์
cd /home/public_html/apiservices/projects/sls_ansi

# รัน simulation
python main.py
```

### Configuration

แก้ไขไฟล์ `data/pattern_1.json` เพื่อปรับแต่ง:

```json
{
  "robots": [{ "id": 1, "name": "R1", "pos": [0, 0] }],
  "packages": [{ "name": "P1", "pickup": [3, 10], "dropoff": [0, 79] }],
  "walls": [[1, 5, 1, 35]],
  "settings": {
    "rows": 26,
    "cols": 80,
    "sleep": 0.5,
    "max_steps": 1000
  }
}
```

---

## 📁 Project Structure

```
sls_ansi/
├── 📄 main.py                        # Entry point & main simulation loop
├── 📁 controllers/
│   └── simulation_controller.py      # Main controller (Composition Pattern)
├── 📁 core/
│   └── settings.py                   # Configuration settings
├── 📁 utils/
│   ├── display_manager.py            # UI/Display & ANSI rendering
│   ├── pathfinding.py                # A* Algorithm & path planning
│   ├── deadlock_resolver.py          # Deadlock detection & resolution
│   ├── robot_manager.py              # Robot & package management
│   ├── grid_utils.py                 # Grid utilities & helpers
│   └── penalty_map.py                # Dynamic penalty system
├── 📁 data/
│   └── pattern_1.json                # Simulation configuration
├── 📁 models/
│   └── deadlock_predictor.pkl        # ML model for deadlock prediction
└── 📁 logs/                          # Runtime logs (auto-generated)
```

---

## 🏗️ Architecture

### System Overview

```mermaid
graph TB
    subgraph Entry["🚀 Entry Point"]
        M[main.py]
    end

    subgraph Controller["🎮 Controller Layer"]
        SC[SimulationController]
    end

    subgraph Modules["🧩 Utility Modules"]
        DM[DisplayManager]
        SR[SimulationRenderer]
        PF[PathFinder]
        DR[DeadlockResolver]
        RM[RobotManager]
    end

    subgraph Core["⚙️ Core"]
        S[Settings]
        GU[GridUtils]
        PM[PenaltyMap]
    end

    subgraph Data["💾 Data"]
        JSON[pattern_1.json]
        ML[deadlock_predictor.pkl]
    end

    M --> SC
    SC --> DM
    SC --> PF
    SC --> DR
    SC --> RM
    DM --> SR

    SC --> S
    PF --> GU
    DR --> GU
    RM --> GU
    SR --> GU

    SC --> JSON
    PF --> ML
```

### Module Dependencies

```mermaid
graph LR
    subgraph Utils["📦 Utils"]
        DM[display_manager]
        PF[pathfinding]
        DR[deadlock_resolver]
        RM[robot_manager]
        GU[grid_utils]
        PM[penalty_map]
    end

    subgraph Core["⚙️ Core"]
        S[settings]
    end

    DM --> S & GU
    PF --> S & GU
    DR --> S & GU
    RM --> S & GU
    GU --> S
```

---

## 🔄 Simulation Workflow

### Main Loop (7 Steps)

```mermaid
flowchart TD
    START([🚀 Start]) --> INIT[Initialize SimulationController]
    INIT --> LOAD[Load Config from JSON]
    LOAD --> INIT_MOD[Initialize Modules]

    INIT_MOD --> LOOP{🔄 Main Loop}

    LOOP --> M1["1️⃣ Maintenance & Cleanup"]
    M1 --> M2["2️⃣ Deadlock Detection"]
    M2 --> M3["3️⃣ Critical Path & Evacuation"]
    M3 --> M4["4️⃣ Decision Making"]
    M4 --> M5["5️⃣ Path Planning"]
    M5 --> M6["6️⃣ Execute Moves"]
    M6 --> M7["7️⃣ Final Path Check"]
    M7 --> RENDER[🖥️ Render Display]

    RENDER --> CHECK{✅ All Delivered?}
    CHECK -->|No| LOOP
    CHECK -->|Yes| STATS[📊 Show Final Statistics]
    STATS --> END([🏁 End])
```

### Step Details

| Step | Name                   | Description                                                       |
| ---- | ---------------------- | ----------------------------------------------------------------- |
| 1    | **Maintenance**        | แก้ไข state ผิดปกติ, ล้าง orphaned assignments, reassign packages |
| 2    | **Deadlock Detection** | ตรวจจับกลุ่ม robots ที่ติดขัดกัน                                  |
| 3    | **Critical Path**      | หลบให้ robots ที่กำลังส่งของ (priority สูงสุด)                    |
| 4    | **Decision Making**    | ตัดสินใจ yield/retreat/emergency สำหรับ robots ที่รอนาน           |
| 5    | **Path Planning**      | หาเส้นทางด้วย A\* Algorithm                                       |
| 6    | **Execute Moves**      | เคลื่อนที่จริง พร้อมตรวจสอบ collision                             |
| 7    | **Final Check**        | ตรวจสอบและแก้ไข path ที่หายไป                                     |

---

## 🤖 Robot State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TO_PICKUP: 📦 Assigned Package
    IDLE --> HOME: 🏠 No Package
    TO_PICKUP --> TO_DROPOFF: ✅ Package Picked
    TO_DROPOFF --> HOME: ✅ Package Delivered
    HOME --> IDLE: 🏠 Reached Home

    IDLE --> EVACUATING: ⚠️ Blocking Path
    TO_PICKUP --> EVACUATING: ⚠️ Need to Yield
    TO_DROPOFF --> EVACUATING: ⚠️ Need to Yield
    HOME --> EVACUATING: ⚠️ Need to Yield
    EVACUATING --> IDLE: ✅ Reached Safe Spot
```

---

## 📦 Package State Machine

```mermaid
stateDiagram-v2
    [*] --> WAITING: 📦 Created
    WAITING --> PICKED: 🤖 Robot Picks Up
    PICKED --> DELIVERED: ✅ Robot Drops Off
    DELIVERED --> [*]
```

---

## 🎯 A\* Algorithm (Enhanced)

### Cost Calculation Formula

```
total_cost = base_cost (1.0)
           + robot_bias (prevent same path selection)
           + ai_deadlock_penalty (ML prediction)
           + turn_penalty (1.5x for turns)
           + dynamic_traffic_cost (future collision prediction)
           + corridor_bonus (prefer wide paths)
           + momentum_bonus (prefer straight lines)
           + goal_proximity_bonus (near goal = lower cost)
           + narrow_passage_penalty (avoid bottlenecks)
```

### Algorithm Flow

```mermaid
flowchart TD
    A[🚀 Start] --> B[Initialize Open Set]
    B --> C{Open Set Empty?}
    C -->|Yes| D[❌ Return Empty Path]
    C -->|No| E[Pop Lowest f-score]
    E --> F{At Goal?}
    F -->|Yes| G[✅ Return Path]
    F -->|No| H[Get Neighbors]
    H --> I[Calculate Enhanced Costs]
    I --> J[Update g_score]
    J --> K[Calculate Heuristic]
    K --> L[Add to Open Set]
    L --> C
```

---

## 🧩 Class Reference

| Class                  | File                       | Description                                 |
| ---------------------- | -------------------------- | ------------------------------------------- |
| `SimulationController` | `simulation_controller.py` | Main orchestrator, uses Composition pattern |
| `PathFinder`           | `pathfinding.py`           | Enhanced A\* with ML deadlock prediction    |
| `DeadlockResolver`     | `deadlock_resolver.py`     | Detect & resolve multi-robot deadlocks      |
| `RobotManager`         | `robot_manager.py`         | Package assignment & robot lifecycle        |
| `DisplayManager`       | `display_manager.py`       | Activity tracking & statistics              |
| `SimulationRenderer`   | `display_manager.py`       | ANSI terminal rendering                     |
| `GridUtils`            | `grid_utils.py`            | Position parsing & grid helpers             |
| `DynamicPenaltyMap`    | `penalty_map.py`           | Traffic-based penalty system                |
| `Settings`             | `settings.py`              | Configuration constants                     |

---

## ⚙️ Configuration Reference

### Settings (settings.py)

| Parameter            | Default | Description                    |
| -------------------- | ------- | ------------------------------ |
| `ROWS`               | 26      | จำนวนแถวของ grid               |
| `COLS`               | 80      | จำนวนคอลัมน์ของ grid           |
| `SLEEP`              | 0.5     | เวลาหน่วง (วินาที) ต่อ step    |
| `MAX_STEPS`          | 1000    | จำนวน step สูงสุด              |
| `YIELD_THRESHOLD`    | 3       | จำนวน wait ก่อนพิจารณา yield   |
| `DEADLOCK_THRESHOLD` | 15      | จำนวน wait ก่อนถือว่า deadlock |
| `TURN_PENALTY`       | 1.5     | cost เพิ่มเมื่อเลี้ยว          |
| `CORRIDOR_BONUS`     | 0.8     | ลด cost เมื่อเดินในทางกว้าง    |

### JSON Config Format

```json
{
  "robots": [
    {
      "id": 1, // Robot ID (unique)
      "name": "R1", // Display name
      "pos": [0, 0] // Initial position [row, col]
    }
  ],
  "packages": [
    {
      "name": "P1", // Package name
      "pickup": [3, 10], // Pickup location
      "dropoff": [0, 79] // Dropoff location
    }
  ],
  "walls": [
    [1, 5, 1, 35] // Wall definition [r1, c1, r2, c2]
  ],
  "settings": {
    "rows": 26,
    "cols": 80,
    "sleep": 0.5,
    "max_steps": 1000
  }
}
```

---

## 📊 Logs

Logs ถูกบันทึกใน `logs/{timestamp}/`:

| File               | Content                        |
| ------------------ | ------------------------------ |
| `system.log`       | Overall system events          |
| `{robot_name}.log` | Per-robot movement & decisions |

---

## 🛠️ Development

### Adding New Robot Behaviors

```python
# ใน RobotManager.fix_robot_states()
def fix_robot_states(self):
    for rb in self.robots:
        # Add your custom state fix logic here
        pass
```

### Customizing A\* Cost

```python
# ใน PathFinder.smart_astar()
# เพิ่ม cost calculation ใหม่:
move_cost += your_custom_penalty * weight
```

### Creating New Patterns

สร้างไฟล์ใหม่ใน `data/`:

```bash
cp data/pattern_1.json data/pattern_2.json
# แก้ไข pattern_2.json
# อัพเดท settings.PATTERN_DIR ใน settings.py
```

---

## 🧠 Model Training

### Deadlock Predictor Model

ระบบใช้ ML Model เพื่อทำนาย deadlock ล่วงหน้า สามารถเทรนใหม่จาก logs ได้

#### Training Commands

```bash
cd /home/public_html/apiservices/projects/sls_ansi

# ใช้ log directory ล่าสุด
python scripts/train_deadlock_model.py

# ใช้ log directory เฉพาะ
python scripts/train_deadlock_model.py --logs-dir logs/20251220_003401

# ใช้ทุก log directories
python scripts/train_deadlock_model.py --all-logs

# กำหนด output path และ cross-validation folds
python scripts/train_deadlock_model.py --output models/new_model.pkl --cv 10
```

#### Training Pipeline

```mermaid
flowchart LR
    A[📁 Log Files] --> B[📖 Parse Logs]
    B --> C[🔧 Feature Engineering]
    C --> D[📊 Train Models]
    D --> E[🏆 Select Best]
    E --> F[💾 Save Model]
```

#### Features Used

| Feature                | Description                                  |
| ---------------------- | -------------------------------------------- |
| `from_row`, `from_col` | ตำแหน่งปัจจุบัน                              |
| `to_row`, `to_col`     | ตำแหน่งเป้าหมาย                              |
| `dir_row`, `dir_col`   | ทิศทางการเคลื่อนที่                          |
| `wait`                 | จำนวน steps ที่รอ                            |
| `state_*`              | State encoding (TO_PICKUP, TO_DROPOFF, etc.) |
| `mode_*`               | Mode encoding (NORMAL, YIELDING, FORCED)     |
| `recent_blocks`        | จำนวน BLOCKED events ล่าสุด                  |
| `recent_moves`         | จำนวน MOVE events ล่าสุด                     |

#### Models Compared

- **RandomForest** - Tree-based ensemble
- **GradientBoosting** - Boosted trees
- **LogisticRegression** - Linear model

Best model is selected by **F1 Score** and saved automatically.

#### Log Format

```
2025-12-20 00:34:05 | MOVE [2, 2] -> [2, 3] | STATE=TO_PICKUP | MODE=NORMAL
2025-12-20 00:34:05 | BLOCKED [5, 3] -> [6, 3] | WAIT=0
2025-12-20 00:34:05 | YIELD to R5 -> [3, 4]
2025-12-20 00:34:05 | RETREAT -> [2, 1]
2025-12-20 00:34:05 | EMERGENCY MOVE -> [1, 3]
```

#### Improving Model Performance

1. **Collect more data** - รัน simulation หลายครั้งเพื่อเพิ่ม samples
2. **Balance classes** - Script มี auto-augmentation สำหรับ deadlock samples
3. **Tune hyperparameters** - แก้ไขใน `train_deadlock_model.py`

---

## 📈 Performance Metrics

เมื่อ simulation จบ จะแสดง:

- **Total Steps** - จำนวน step ทั้งหมด
- **Elapsed Time** - เวลาที่ใช้จริง
- **Packages Delivered** - จำนวน package ที่ส่งสำเร็จ
- **Total Moves** - จำนวนการเคลื่อนที่ทั้งหมด
- **Deadlocks Resolved** - จำนวน deadlock ที่แก้ไข
- **Delivery Rate** - อัตราการส่ง (% per step)
- **Moves per Package** - จำนวน moves เฉลี่ยต่อ package

---

## 🧪 Route System Experiments

ได้ทดลองระบบ Route System หลายแบบเพื่อให้ Robot เดินเป็นธรรมชาติมากขึ้น:

### แบบที่ทดลอง

| #   | แบบ                   | คำอธิบาย                                                          |
| --- | --------------------- | ----------------------------------------------------------------- |
| 1   | **Original**          | A\* Algorithm แบบเดิม ไม่มี Route System                          |
| 2   | **Full Route System** | ใช้ RouteAnalyzer + Highway Bonus เต็มรูปแบบ                      |
| 3   | **Reduced Bonus**     | ลด Highway Bonus จาก 0.08 → 0.03                                  |
| 4   | **Hybrid**            | ใช้ Route System เมื่อไม่ติดขัด, switch เป็นแบบเดิมเมื่อ wait > 0 |
| 5   | **Smart Hybrid**      | Hybrid + ปรับปรุง route optimization                              |
| 6   | **Time-Space A\***    | A\* ในมิติ space-time พร้อม WAIT action และ Reservation Table     |

### ผลการทดลอง

| Metric        | Original | Time-Space A\* | Hybrid  | **Smart Hybrid** |
| ------------- | -------- | -------------- | ------- | ---------------- |
| Total Steps   | 387      | 385            | 387     | **369** ✅       |
| Total Moves   | 5,498    | **4,998** ✅   | 5,182   | 5,489            |
| Total Turns   | 577      | 541            | **468** | 539              |
| Yield Events  | **6**    | 21             | 8       | 10               |
| Deadlocks     | 0        | 0              | 0       | 0                |
| Moves/Package | 219.9    | **199.9** ✅   | 207.3   | 219.6            |
| Elapsed Time  | 04:12    | 04:16          | 03:45   | **03:40** ✅     |
| Delivery Rate | 6.46%    | 6.49%          | 6.46%   | **6.78%** ✅     |

### สรุปผล

| Algorithm          | จุดเด่น                       | เหมาะสำหรับ            |
| ------------------ | ----------------------------- | ---------------------- |
| **Smart Hybrid**   | เร็วที่สุด, Delivery Rate สูง | Production, Real-time  |
| **Time-Space A\*** | Moves น้อยสุด, ประหยัดพลังงาน | Battery-powered robots |

---

## ⏱️ Time-Space A\* Algorithm

### Overview

Time-Space A\* ขยาย search space ไปยังมิติเวลา โดย node = `(position, time, direction)`

```
┌─────────────────────────────────────────────────────────────┐
│                    Time-Space A* vs Normal A*               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Normal A*:     State = (position)                         │
│   Time-Space A*: State = (position, time, direction)        │
│                                                             │
│   Actions:                                                  │
│   ├── MOVE_UP, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT             │
│   └── WAIT (รอที่เดิม 1 timestep)                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Toggle การใช้งาน

```python
# ใน core/settings.py
settings.USE_TIME_SPACE_ASTAR = False  # Smart Hybrid (default)
settings.USE_TIME_SPACE_ASTAR = True   # Time-Space A*
```

### Time-Space A\* Settings

| Parameter              | Default | Description                        |
| ---------------------- | ------- | ---------------------------------- |
| `USE_TIME_SPACE_ASTAR` | False   | เปิด/ปิด Time-Space A\*            |
| `TIME_HORIZON`         | 30      | จำนวน timesteps สูงสุดที่จะ plan   |
| `MAX_WAIT_ACTIONS`     | 5       | จำนวนครั้งสูงสุดที่ WAIT ติดต่อกัน |
| `WAIT_COST`            | 1.2     | cost ของการ WAIT                   |

### Reservation Table

```mermaid
sequenceDiagram
    participant R1 as Robot 1
    participant RT as ReservationTable
    participant R2 as Robot 2

    R1->>RT: find_path() + reserve
    RT-->>R1: Path [(0,0)@t0, (0,1)@t1, (0,2)@t2]

    R2->>RT: find_path() (check reservations)
    Note over RT: (0,1)@t1 is reserved by R1
    RT-->>R2: Alternative path or WAIT
```

### กลไก Hybrid System

```
┌──────────────────────────────────────────────────────────────┐
│                      Robot State Check                       │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│   wait_count == 0 ?                                          │
│         │                                                    │
│         ├── Yes ──► Use Route System                         │
│         │           • Highway Bonus                          │
│         │           • Main Corridor Bonus                    │
│         │           • Preferred Direction                    │
│         │           • Route Cache                            │
│         │                                                    │
│         └── No ───► Use Original A*                          │
│                     • Momentum-based sorting                 │
│                     • Invalidate cache                       │
│                     • Skip Route bonuses                     │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

---

## 🤝 Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## 📝 License

MIT License - see [LICENSE](LICENSE) for details.

---

## 👤 Author

**TiwKill**

---

<p align="center">
  Made with ❤️ and 🐍 Python
</p>
