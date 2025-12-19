"""
Pathfinding Module
จัดการการหาเส้นทาง A* และ Path Planning สำหรับ Smart Logistics Simulation
"""

import heapq
from core.settings import settings
from utils.grid_utils import GridUtils


class PathFinder:
    """จัดการการหาเส้นทางด้วย A* Algorithm"""
    
    def __init__(self, obstacles, corridor_map, robots, packages, deadlock_model=None):
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.robots = robots
        self.packages = packages
        self.deadlock_model = deadlock_model
    
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
        
        # ตรวจสอบว่าตำแหน่งนี้จะมีหุ่นยนต์อื่นมาชนหรือไม่
        for rid, future_positions in future_predictions.items():
            if rid == robot["id"]:
                continue
            
            for step_idx, future_pos in enumerate(future_positions):
                if future_pos == pos:
                    # ยิ่งใกล้ในอนาคต penalty ยิ่งสูง
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
        
        # นับจำนวนทิศทางที่เปิด
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if GridUtils.in_bounds(nr, nc) and (nr, nc) not in self.obstacles:
                open_count += 1
        
        # ถ้าเปิดแค่ 2 ทาง = ทางแคบ
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
        """A* Algorithm พร้อม Enhanced Cost Calculation"""
        if start == goal:
            return []
        
        # ทำนายตำแหน่งอนาคตของหุ่นยนต์อื่น
        future_predictions = self.predict_future_positions(robot, steps=5)
        
        open_set = [(0, 0, start, robot["last_dir"], [])]
        came_from = {}
        g_score = {(start, robot["last_dir"]): 0}
        
        while open_set:
            _, g, current, last_dir, path = heapq.heappop(open_set)
            
            if current == goal:
                return path + [current] if current != start else path
            
            state = (current, last_dir)
            if state in came_from:
                continue
            came_from[state] = True
            
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            # เรียงลำดับทิศทางตาม momentum
            if last_dir != (0, 0):
                directions.sort(key=lambda d: 0 if d == last_dir else 1)
            
            for dr, dc in directions:
                nr, nc = current[0] + dr, current[1] + dc
                nxt = (nr, nc)
                new_dir = (dr, dc)
                
                if not GridUtils.in_bounds(nr, nc) or nxt in blocked:
                    continue
                
                if nxt != goal and not self.can_enter_dropoff(robot, nxt):
                    continue
                if nxt != goal and not self.can_enter_pickup(robot, nxt):
                    continue
                
                # =========================
                # ENHANCED COST CALCULATION
                # =========================
                move_cost = 1.0
                
                # 1. Robot-specific bias (ป้องกันการเลือกทางเดียวกัน)
                robot_bias = (robot["id"] % 3) * 0.15
                move_cost += robot_bias
                
                # 2. AI Deadlock Prediction
                risk = 0.0
                wait = robot.get("wait_count", 0)
                robot_state = robot.get("state")
                
                if wait >= 5 and robot_state != "IDLE" and self.deadlock_model:
                    try:
                        features = self.build_deadlock_features(robot, current, nxt)
                        risk = self.deadlock_model.predict_proba(features)[0][1]
                        
                        AI_WEIGHT = 2.0
                        AI_MAX_PENALTY = 1.5
                        
                        penalty = min(risk * AI_WEIGHT, AI_MAX_PENALTY)
                        
                        # Corridor bias: ให้ทิศเดิม penalty ต่ำลง
                        if new_dir == robot.get("last_dir"):
                            penalty *= 0.3
                        
                        move_cost += penalty
                    except Exception:
                        pass
                
                # 3. Turn Penalty
                if GridUtils.is_turn(last_dir, new_dir) and last_dir != (0, 0):
                    move_cost += settings.TURN_PENALTY
                
                # 4. Dynamic Traffic Cost (ใช้การทำนายอนาคต)
                dynamic_traffic = self.get_dynamic_traffic_cost(nxt, robot, future_predictions)
                move_cost += dynamic_traffic * 0.15
                
                # 5. Corridor Bonus
                corridor_score = self.corridor_map.get(nxt, 0)
                if corridor_score >= 6:
                    move_cost *= settings.CORRIDOR_BONUS
                elif corridor_score <= 2:
                    move_cost *= 1.3
                
                # 6. Momentum Bonus
                if not GridUtils.is_turn(last_dir, new_dir) and robot["momentum"] > 0:
                    move_cost *= max(0.65, 1.0 - robot["momentum"] * 0.06)
                
                # 7. Goal Proximity Bonus
                dist_to_goal = GridUtils.manhattan(nxt, goal)
                if dist_to_goal <= 3:
                    move_cost *= 0.9
                
                # 8. Narrow Passage Detection
                if self.is_narrow_passage(nxt):
                    priority = self.get_robot_priority(robot)
                    if priority < 2000:
                        move_cost *= 1.5
                
                new_g = g + move_cost
                new_state = (nxt, new_dir)
                
                if new_state not in g_score or new_g < g_score[new_state]:
                    g_score[new_state] = new_g
                    h = GridUtils.manhattan(nxt, goal)
                    
                    # Heuristic Improvement
                    goal_dir = (
                        1 if goal[0] > nxt[0] else (-1 if goal[0] < nxt[0] else 0),
                        1 if goal[1] > nxt[1] else (-1 if goal[1] < nxt[1] else 0)
                    )
                    
                    if new_dir[0] == goal_dir[0] or new_dir[1] == goal_dir[1]:
                        h *= 0.92
                    
                    if robot["momentum"] >= 3 and new_dir == last_dir:
                        h *= 0.95
                    
                    f = new_g + h
                    new_path = path + [nxt]
                    heapq.heappush(open_set, (f, new_g, nxt, new_dir, new_path))
        
        return []

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
