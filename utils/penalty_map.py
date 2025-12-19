import json
import os
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Tuple, Set
import numpy as np

@dataclass
class CellPenalty:
    """เก็บค่าปรับของแต่ละเซลล์"""
    base_penalty: float = 0.0
    traffic_history: int = 0  # จำนวนครั้งที่มีการจราจร
    conflict_history: int = 0  # จำนวนครั้งที่เกิดความขัดแย้ง
    last_updated: int = 0  # step ล่าสุดที่อัพเดท
    yield_zone: bool = False  # เป็นโซนที่ควรหลบให้
    priority_zone: bool = False  # เป็นโซนที่มีสิทธิ์สูง

class DynamicPenaltyMap:
    """จัดการแผนที่ค่าปรับแบบไดนามิก"""
    
    def __init__(self, rows: int, cols: int):
        self.rows = rows
        self.cols = cols
        self.cells: Dict[Tuple[int, int], CellPenalty] = {}
        self.history_file = "penalty_history.json"
        
        # เริ่มต้นทุกเซลล์
        for r in range(rows):
            for c in range(cols):
                self.cells[(r, c)] = CellPenalty()
    
    def update_traffic(self, pos: Tuple[int, int], step: int, weight: float = 1.0):
        """อัพเดทประวัติการจราจร"""
        if pos in self.cells:
            cell = self.cells[pos]
            cell.traffic_history += 1
            cell.base_penalty += weight * 0.05
            cell.base_penalty = min(cell.base_penalty, 2.0)  # จำกัดสูงสุด
            cell.last_updated = step
    
    def update_conflict(self, pos: Tuple[int, int], step: int, severity: float = 1.0):
        """อัพเดทประวัติความขัดแย้ง"""
        if pos in self.cells:
            cell = self.cells[pos]
            cell.conflict_history += 1
            cell.base_penalty += severity * 0.1
            cell.base_penalty = min(cell.base_penalty, 3.0)
            cell.last_updated = step
    
    def mark_yield_zone(self, pos: Tuple[int, int], duration: int = 10):
        """ทำเครื่องหมายเป็นโซนหลบ"""
        if pos in self.cells:
            self.cells[pos].yield_zone = True
            # รีเซ็ตหลังจาก duration step
            if duration > 0:
                self.cells[pos]._yield_expire = duration
    
    def mark_priority_zone(self, pos: Tuple[int, int], duration: int = 20):
        """ทำเครื่องหมายเป็นโซนสิทธิ์สูง"""
        if pos in self.cells:
            self.cells[pos].priority_zone = True
            if duration > 0:
                self.cells[pos]._priority_expire = duration
    
    def get_penalty(self, pos: Tuple[int, int], robot_state: str = None) -> float:
        """คำนวณค่าปรับรวม"""
        if pos not in self.cells:
            return 0.0
        
        cell = self.cells[pos]
        penalty = cell.base_penalty
        
        # เพิ่มค่าปรับตามสถานะ
        if robot_state == "TO_DROPOFF" and cell.yield_zone:
            penalty += 2.0  # ห้ามเข้า yield zone ถ้ากำลังส่งของ
        elif robot_state == "IDLE" and cell.priority_zone:
            penalty += 1.5  # IDLE หลบ priority zone
        
        return min(penalty, 5.0)  # จำกัดสูงสุด
    
    def step_update(self, current_step: int):
        """อัพเดทสถานะแต่ละ step"""
        for pos, cell in self.cells.items():
            # ลดค่าปรับตามเวลา
            if current_step - cell.last_updated > 50:
                cell.base_penalty *= 0.95
            
            # รีเซ็ต yield zone
            if hasattr(cell, '_yield_expire'):
                cell._yield_expire -= 1
                if cell._yield_expire <= 0:
                    cell.yield_zone = False
                    delattr(cell, '_yield_expire')
            
            # รีเซ็ต priority zone
            if hasattr(cell, '_priority_expire'):
                cell._priority_expire -= 1
                if cell._priority_expire <= 0:
                    cell.priority_zone = False
                    delattr(cell, '_priority_expire')
    
    def get_congestion_map(self, radius: int = 3) -> Dict[Tuple[int, int], float]:
        """คำนวณความหนาแน่นในพื้นที่"""
        congestion = {}
        for (r, c), cell in self.cells.items():
            density = 0
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in self.cells:
                        neighbor = self.cells[(nr, nc)]
                        distance = max(abs(dr), abs(dc))
                        weight = 1.0 / (distance + 1)
                        density += (neighbor.traffic_history * 0.1 + 
                                  neighbor.conflict_history * 0.2) * weight
            congestion[(r, c)] = density
        return congestion
    
    def save_history(self, filename: str = None):
        """บันทึกประวัติ"""
        if filename is None:
            filename = self.history_file
        
        data = {
            'cells': {},
            'metadata': {
                'rows': self.rows,
                'cols': self.cols,
                'total_cells': len(self.cells)
            }
        }
        
        for pos, cell in self.cells.items():
            pos_str = f"{pos[0]}_{pos[1]}"
            data['cells'][pos_str] = {
                'base_penalty': cell.base_penalty,
                'traffic_history': cell.traffic_history,
                'conflict_history': cell.conflict_history,
                'yield_zone': cell.yield_zone,
                'priority_zone': cell.priority_zone
            }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
    
    def load_history(self, filename: str = None):
        """โหลดประวัติ"""
        if filename is None:
            filename = self.history_file
        
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                data = json.load(f)
            
            for pos_str, cell_data in data['cells'].items():
                r, c = map(int, pos_str.split('_'))
                pos = (r, c)
                if pos in self.cells:
                    self.cells[pos].base_penalty = cell_data.get('base_penalty', 0.0)
                    self.cells[pos].traffic_history = cell_data.get('traffic_history', 0)
                    self.cells[pos].conflict_history = cell_data.get('conflict_history', 0)
                    self.cells[pos].yield_zone = cell_data.get('yield_zone', False)
                    self.cells[pos].priority_zone = cell_data.get('priority_zone', False)