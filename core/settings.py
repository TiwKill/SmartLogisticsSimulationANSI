# ======================
# CONFIG
# ======================

import os

class Settings:
    def __init__(self):
        self.BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        self.PATTERN_DIR = os.path.join(self.BASE_DIR, "../data/pattern_1.json")
        self.DEADLOCK_MODEL_PATH = os.path.join(self.BASE_DIR, "../models/deadlock_predictor.pkl")
        self.LOG_DIR = os.path.join(self.BASE_DIR, "../logs")

        # Grid Settings (can be overridden by JSON config)
        self.ROWS = 26
        self.COLS = 80
        self.SLEEP = 0.5
        self.MAX_WAIT = 300
        self.MAX_STEPS = 1000

        self.TURN_PENALTY = 1.5          # cost เพิ่มเมื่อเลี้ยว
        self.TRAFFIC_PENALTY = 3.0       # cost เพิ่มเมื่อใกล้ robot อื่น
        self.CORRIDOR_BONUS = 0.8        # ลด cost เมื่อเดินในทางกว้าง
        self.PREDICTION_STEPS = 4        # ดูล่วงหน้ากี่ step
        self.SMOOTHING_WEIGHT = 0.3      # น้ำหนักสำหรับ path smoothing

        self.DECISION_WAIT_THRESHOLD = 5       # จำนวน wait ก่อนเปลี่ยน decision
        self.FORCE_MOVE_THRESHOLD = 10         # จำนวน wait ก่อน force move
        self.DEADLOCK_THRESHOLD = 15           # จำนวน wait ก่อนถือว่า deadlock
        self.YIELD_THRESHOLD = 3               # จำนวน wait ก่อนพิจารณา yield

        self.REASSIGN_THRESHOLD = 30           # จำนวน wait ก่อน reassign package
        self.IDLE_RECHECK_INTERVAL = 1         # ทุกๆ N steps ให้ IDLE robot ลองหา package ใหม่
        self.ORPHAN_CHECK_INTERVAL = 1         # ทุกๆ N steps ให้ตรวจ orphaned packages

        self.PENALTY_DECAY_RATE = 0.95  # อัตราการลดค่าปรับ
        self.TRAFFIC_WEIGHT = 0.05      # น้ำหนักการจราจร
        self.CONFLICT_WEIGHT = 0.1      # น้ำหนักความขัดแย้ง
        self.MAX_CELL_PENALTY = 5.0     # ค่าปรับสูงสุดต่อเซลล์
    
        # Yield/Priority Policy
        self.YIELD_ZONE_DURATION = 10   # ระยะเวลาโซนหลบ (step)
        self.PRIORITY_ZONE_DURATION = 20 # ระยะเวลาโซนสิทธิ์สูง
        self.MIN_PRIORITY_DIFF = 200    # ความต่างขั้นต่ำสำหรับ priority
        self.YIELD_DISTANCE_THRESHOLD = 2  # ระยะห่างสำหรับ yield
    
        # Congestion Detection
        self.CONGESTION_RADIUS = 3      # รัศมีตรวจสอบความหนาแน่น
        self.CONGESTION_THRESHOLD = 3.0 # ค่าความหนาแน่นที่ถือว่าติดขัด

        # Time-Space A* Settings
        self.USE_TIME_SPACE_ASTAR = False  # True = Time-Space A*, False = Smart Hybrid
        self.TIME_HORIZON = 30           # จำนวน timesteps สูงสุดที่จะ plan
        self.MAX_WAIT_ACTIONS = 5        # จำนวนครั้งสูงสุดที่ WAIT ติดต่อกัน
        self.WAIT_COST = 1.2             # cost ของการ WAIT (สูงกว่า MOVE เล็กน้อย)

settings = Settings()