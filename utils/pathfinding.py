"""
Pathfinding Module
จัดการการหาเส้นทาง Time-Space A* สำหรับ Smart Logistics Simulation
"""

from core.settings import settings
from utils.grid_utils import GridUtils
from utils.time_space_astar import TimeSpaceAStar, ReservationTable


class PathFinder:
    """จัดการการหาเส้นทางด้วย Time-Space A* Algorithm"""
    
    def __init__(self, obstacles, corridor_map, robots, packages, 
                 deadlock_model=None, route_analyzer=None, route_cache=None):
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.robots = robots
        self.packages = packages
        self.deadlock_model = deadlock_model
        self.route_analyzer = route_analyzer
        self.route_cache = route_cache
        
        # Time-Space A* components
        self.reservation_table = ReservationTable()
        self.ts_astar = TimeSpaceAStar(
            obstacles=obstacles,
            corridor_map=corridor_map,
            robots=robots,
            packages=packages,
            reservation_table=self.reservation_table,
            deadlock_model=deadlock_model,
            route_analyzer=route_analyzer,
            route_cache=route_cache
        )
        
        # Current simulation step (ต้อง update ทุก step)
        self.current_step = 0
    
    def update_step(self, step):
        """อัพเดท current step และล้าง old reservations"""
        self.current_step = step
        self.reservation_table.clear_old(step)
    
    def reserve_robot_path(self, robot, path):
        """จอง path สำหรับ robot"""
        if path:
            self.reservation_table.reserve_path(robot["id"], path, self.current_step)
    
    def clear_robot_reservations(self, robot):
        """ล้างการจองของ robot"""
        self.reservation_table.clear_robot(robot["id"])
    
    def predict_future_positions(self, robot, steps=3):
        """ทำนายตำแหน่งของ robot อื่นในอนาคต"""
        predictions = {}
        for other in self.robots:
            if other["id"] == robot["id"]:
                continue
            
            future_pos = [other["pos"]]
            for i in range(min(steps, len(other["path"]))):
                future_pos.append(other["path"][i])
            predictions[other["id"]] = future_pos
        return predictions

    def get_dynamic_traffic_cost(self, pos, robot, future_predictions):
        """คำนวณ traffic cost แบบ dynamic รวมทั้งทำนายอนาคต"""
        cost = 0.0
        
        for rid, future_positions in future_predictions.items():
            if rid == robot["id"]:
                continue
            
            for step_idx, future_pos in enumerate(future_positions):
                if future_pos == pos:
                    cost += 10.0 / (step_idx + 1)
                else:
                    dist = GridUtils.manhattan(pos, future_pos)
                    if dist <= 2:
                        cost += 2.0 / ((step_idx + 1) * dist)
        
        return cost

    def build_deadlock_features(self, robot, curr, nxt):
        """สร้าง features สำหรับ deadlock prediction"""
        import pandas as pd
        return pd.DataFrame([{
            "from_row": curr[0],
            "from_col": curr[1],
            "to_row": nxt[0],
            "to_col": nxt[1],
            "wait": robot.get("wait_count", 0)
        }])

    def is_narrow_passage(self, pos):
        """ตรวจสอบว่าตำแหน่งนี้เป็นทางแคบหรือไม่"""
        r, c = pos
        open_count = 0
        
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if GridUtils.in_bounds(nr, nc) and (nr, nc) not in self.obstacles:
                open_count += 1
        
        return open_count <= 2

    def can_enter_dropoff(self, robot, pos):
        """ตรวจสอบสิทธิ์การเข้าจุด Dropoff"""
        for pid, pkg in self.packages.items():
            if pkg["dropoff"] == pos:
                if robot["package"] == pid and robot["state"] == "TO_DROPOFF":
                    return True
                if pkg["status"] == "DELIVERED":
                    return True
                if pkg["status"] == "WAITING":
                    return True
                return False
        return True

    def can_enter_pickup(self, robot, pos):
        """ตรวจสอบสิทธิ์การเข้าจุด Pickup"""
        if robot["package"] is not None:
            my_pkg = self.packages[robot["package"]]
            if my_pkg["pickup"] == pos and robot["state"] == "TO_PICKUP":
                return True

        for pid, pkg in self.packages.items():
            if pkg["pickup"] == pos:
                if pkg["status"] == "WAITING":
                    return False
        
        return True

    def get_robot_priority(self, robot):
        """คำนวณ priority ของ robot"""
        state_priority = {
            "TO_DROPOFF": 3000,
            "TO_PICKUP": 2000,
            "EVACUATING": 1500,
            "HOME": 1000,
            "IDLE": 0
        }
        base = state_priority.get(robot["state"], 0)
        wait_bonus = robot.get("wait_count", 0) * 100
        dist_bonus = 0
        if robot["path"]:
            dist_bonus = 500 - min(len(robot["path"]), 500)
        momentum_bonus = robot.get("momentum", 0) * 50
        return base + wait_bonus + dist_bonus + momentum_bonus

    def smart_astar(self, start, goal, blocked, robot):
        """Smart A* Algorithm with Toggle
        
        ใช้ Time-Space A* หรือ Smart Hybrid ตาม settings.USE_TIME_SPACE_ASTAR
        - Time-Space A*: หลีกเลี่ยงการชนในมิติเวลา, สามารถ WAIT ได้
        - Smart Hybrid: เร็วกว่า, ใช้ route optimization
        """
        if settings.USE_TIME_SPACE_ASTAR:
            # Time-Space A* mode
            path = self.ts_astar.find_path(start, goal, self.current_step, robot, blocked)
            
            # จอง path ถ้าหาเจอ
            if path:
                self.reserve_robot_path(robot, path)
            
            return path
        else:
            # Smart Hybrid mode (fallback A* with route optimization)
            return self.ts_astar._fallback_astar(start, goal, robot, blocked)

    def smooth_path(self, path, robot):
        """
        ปิดการ smooth path เพื่อให้ robot เดินทีละ node เท่านั้น
        ไม่ข้าม node ไปเลย - ทุก robot เดินความเร็วเท่ากัน
        """
        return path

    def has_clear_line(self, start, end, robot):
        """ตรวจสอบว่ามีเส้นทางตรงที่ชัดเจนหรือไม่"""
        r0, c0 = start
        r1, c1 = end
        
        if start == end:
            return True
        
        if abs(r1 - r0) + abs(c1 - c0) > 5:
            return False
        
        steps = max(abs(r1 - r0), abs(c1 - c0))
        
        if steps == 0:
            return True
        
        for step in range(1, steps):
            t = step / steps
            r = int(r0 + (r1 - r0) * t)
            c = int(c0 + (c1 - c0) * t)
            pos = (r, c)
            
            if not GridUtils.in_bounds(r, c):
                return False
            if pos in self.obstacles:
                return False
            if not self.can_enter_dropoff(robot, pos):
                return False
            if not self.can_enter_pickup(robot, pos):
                return False
        
        return True
