"""
Time-Space A* Pathfinding Module
================================

ใช้ A* Algorithm ในมิติ space-time เพื่อหลีกเลี่ยงการชนกัน
- State = (position, time, direction)
- Actions = MOVE_UP, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT, WAIT
"""

import heapq
from collections import defaultdict
from core.settings import settings
from utils.grid_utils import GridUtils


class ReservationTable:
    """ตารางจองตำแหน่งในแต่ละ timestep"""
    
    def __init__(self):
        # {timestep: {position: robot_id}}
        self.reservations = defaultdict(dict)
        # {robot_id: [(position, timestep), ...]}
        self.robot_reservations = defaultdict(list)
    
    def reserve(self, robot_id, position, timestep):
        """จองตำแหน่งในเวลาที่กำหนด"""
        self.reservations[timestep][position] = robot_id
        self.robot_reservations[robot_id].append((position, timestep))
    
    def reserve_path(self, robot_id, path, start_time):
        """จอง path ทั้งหมดตั้งแต่ start_time"""
        self.clear_robot(robot_id)
        
        current_time = start_time
        for pos in path:
            self.reserve(robot_id, pos, current_time)
            current_time += 1
        
        # จองตำแหน่งสุดท้ายไปอีก TIME_HORIZON เพื่อป้องกันการชนหลังถึง
        if path:
            last_pos = path[-1]
            for t in range(current_time, current_time + settings.TIME_HORIZON):
                self.reserve(robot_id, last_pos, t)
    
    def is_reserved(self, position, timestep, exclude_robot=None):
        """ตรวจสอบว่าตำแหน่งถูกจองในเวลานั้นหรือไม่"""
        if timestep in self.reservations:
            if position in self.reservations[timestep]:
                reserved_by = self.reservations[timestep][position]
                if exclude_robot is not None and reserved_by == exclude_robot:
                    return False
                return True
        return False
    
    def get_reserved_by(self, position, timestep):
        """ดูว่าใครจองตำแหน่งนี้"""
        if timestep in self.reservations:
            return self.reservations[timestep].get(position)
        return None
    
    def clear_robot(self, robot_id):
        """ล้างการจองของหุ่นยนต์"""
        for pos, timestep in self.robot_reservations[robot_id]:
            if timestep in self.reservations:
                if self.reservations[timestep].get(pos) == robot_id:
                    del self.reservations[timestep][pos]
        self.robot_reservations[robot_id] = []
    
    def clear_old(self, current_time):
        """ล้างการจองที่ผ่านไปแล้ว"""
        old_times = [t for t in self.reservations.keys() if t < current_time]
        for t in old_times:
            del self.reservations[t]
        
        # ล้าง robot_reservations ด้วย
        for robot_id in self.robot_reservations:
            self.robot_reservations[robot_id] = [
                (pos, t) for pos, t in self.robot_reservations[robot_id] 
                if t >= current_time
            ]


class TimeSpaceAStar:
    """Time-Space A* Pathfinder"""
    
    def __init__(self, obstacles, corridor_map, robots, packages, 
                 reservation_table=None, deadlock_model=None, 
                 route_analyzer=None, route_cache=None):
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.robots = robots
        self.packages = packages
        self.reservation_table = reservation_table or ReservationTable()
        self.deadlock_model = deadlock_model
        self.route_analyzer = route_analyzer
        self.route_cache = route_cache
    
    def find_path(self, start, goal, start_time, robot, blocked=None):
        """
        Time-Space A* Algorithm
        
        Args:
            start: ตำแหน่งเริ่มต้น (row, col)
            goal: ตำแหน่งเป้าหมาย (row, col)
            start_time: timestep เริ่มต้น
            robot: ข้อมูล robot
            blocked: ตำแหน่งที่ถูก block (static obstacles)
        
        Returns:
            list of positions (path) หรือ [] ถ้าหาไม่เจอ
        """
        if start == goal:
            return []
        
        if blocked is None:
            blocked = set()
        
        # ตรวจสอบว่า robot ติดขัดหรือไม่
        is_stuck = robot.get("wait_count", 0) > 0
        use_route_system = self.route_analyzer and not is_stuck
        
        # ลองใช้ cached route ก่อน (ถ้ามี และไม่ติดขัด)
        if self.route_cache and not is_stuck:
            cached_path = self.route_cache.get(start, goal, robot.get("state", "IDLE"))
            if cached_path:
                # ตรวจสอบว่า cached path ยังใช้ได้
                if not any(p in blocked for p in cached_path):
                    # ตรวจสอบ reservations
                    path_valid = True
                    for i, pos in enumerate(cached_path):
                        if self.reservation_table.is_reserved(pos, start_time + i, robot["id"]):
                            path_valid = False
                            break
                    if path_valid:
                        return cached_path
        
        # ถ้าติดขัด ให้ invalidate cache
        if is_stuck and self.route_cache:
            self.route_cache.invalidate([start])
        
        # A* Search in Time-Space
        # State: (position, time, last_direction)
        # Priority queue: (f_score, g_score, position, time, last_dir, path)
        
        open_set = [(0, 0, start, start_time, robot["last_dir"], [])]
        came_from = {}
        g_score = {(start, start_time, robot["last_dir"]): 0}
        
        max_time = start_time + settings.TIME_HORIZON
        
        while open_set:
            _, g, current, current_time, last_dir, path = heapq.heappop(open_set)
            
            # ถึงเป้าหมายแล้ว
            if current == goal:
                result_path = path + [current] if current != start else path
                
                # Cache the result
                if self.route_cache and len(result_path) > 0 and not is_stuck:
                    self.route_cache.put(start, goal, robot.get("state", "IDLE"), result_path)
                
                return result_path
            
            # ถ้าเวลาเกิน horizon ให้ข้าม
            if current_time >= max_time:
                continue
            
            state = (current, current_time, last_dir)
            if state in came_from:
                continue
            came_from[state] = True
            
            # Generate successors: 4 directions + WAIT
            # Actions: MOVE_UP, MOVE_DOWN, MOVE_LEFT, MOVE_RIGHT, WAIT
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            
            # ใช้ RouteAnalyzer เฉพาะเมื่อไม่ติดขัด
            if use_route_system:
                preferred = self.route_analyzer.get_preferred_direction(current, goal, robot.get("state", "IDLE"))
                if preferred != (0, 0):
                    directions.sort(key=lambda d: 0 if d == preferred else (1 if d == last_dir else 2))
                elif last_dir != (0, 0):
                    directions.sort(key=lambda d: 0 if d == last_dir else 1)
            elif last_dir != (0, 0):
                directions.sort(key=lambda d: 0 if d == last_dir else 1)
            
            next_time = current_time + 1
            
            # === MOVE Actions ===
            for dr, dc in directions:
                nr, nc = current[0] + dr, current[1] + dc
                nxt = (nr, nc)
                new_dir = (dr, dc)
                
                # ตรวจสอบ bounds และ obstacles
                if not GridUtils.in_bounds(nr, nc) or nxt in self.obstacles or nxt in blocked:
                    continue
                
                # ตรวจสอบสิทธิ์เข้า dropoff/pickup
                if nxt != goal and not self.can_enter_dropoff(robot, nxt):
                    continue
                if nxt != goal and not self.can_enter_pickup(robot, nxt):
                    continue
                
                # ตรวจสอบ reservation (Time-Space collision avoidance)
                if self.reservation_table.is_reserved(nxt, next_time, robot["id"]):
                    continue
                
                # ตรวจสอบ edge collision (swap positions)
                # ถ้ามีหุ่นยนต์อื่นจะย้ายมาที่ current ในเวลา next_time
                if self._will_swap(current, nxt, current_time, robot["id"]):
                    continue
                
                # คำนวณ cost
                move_cost = self._calculate_move_cost(robot, current, nxt, last_dir, new_dir, use_route_system)
                
                new_g = g + move_cost
                new_state = (nxt, next_time, new_dir)
                
                if new_state not in g_score or new_g < g_score[new_state]:
                    g_score[new_state] = new_g
                    h = GridUtils.manhattan(nxt, goal)
                    
                    # Heuristic improvements
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
                    heapq.heappush(open_set, (f, new_g, nxt, next_time, new_dir, new_path))
            
            # === WAIT Action ===
            # นับจำนวน consecutive waits ใน path
            consecutive_waits = 0
            for p in reversed(path):
                if p == current:
                    consecutive_waits += 1
                else:
                    break
            
            # ถ้ายังไม่เกิน MAX_WAIT_ACTIONS ให้ลอง WAIT
            if consecutive_waits < settings.MAX_WAIT_ACTIONS:
                # WAIT = อยู่ที่เดิม ไปเวลาถัดไป
                # ตรวจสอบว่ายังอยู่ที่เดิมได้หรือไม่
                if not self.reservation_table.is_reserved(current, next_time, robot["id"]):
                    wait_cost = settings.WAIT_COST
                    new_g_wait = g + wait_cost
                    wait_state = (current, next_time, last_dir)
                    
                    if wait_state not in g_score or new_g_wait < g_score[wait_state]:
                        g_score[wait_state] = new_g_wait
                        h = GridUtils.manhattan(current, goal)
                        f = new_g_wait + h
                        
                        # path ยังคงเป็น current (WAIT ไม่เพิ่ม position ใหม่ แต่อยู่ที่เดิม)
                        # เราจะเก็บ current ซ้ำเพื่อแสดงว่า WAIT
                        new_path_wait = path + [current]
                        heapq.heappush(open_set, (f, new_g_wait, current, next_time, last_dir, new_path_wait))
        
        # ถ้าหาไม่เจอใน time-space ให้ fallback ไป basic A*
        return self._fallback_astar(start, goal, robot, blocked)
    
    def _will_swap(self, current, nxt, current_time, robot_id):
        """ตรวจสอบว่าจะเกิด swap หรือไม่"""
        # ถ้ามีหุ่นยนต์ที่ nxt ใน current_time และจะย้ายไป current ใน next_time
        other_id = self.reservation_table.get_reserved_by(nxt, current_time)
        if other_id and other_id != robot_id:
            # ตรวจสอบว่า other จะย้ายมาที่ current หรือไม่
            if self.reservation_table.get_reserved_by(current, current_time + 1) == other_id:
                return True
        return False
    
    def _calculate_move_cost(self, robot, current, nxt, last_dir, new_dir, use_route_system):
        """คำนวณ cost ของการเคลื่อนที่ (เหมือน smart_astar เดิม)"""
        move_cost = 1.0
        
        # 1. Robot-specific bias
        robot_bias = (robot["id"] % 3) * 0.15
        move_cost += robot_bias
        
        # 2. Turn Penalty
        if GridUtils.is_turn(last_dir, new_dir) and last_dir != (0, 0):
            move_cost += settings.TURN_PENALTY * 0.7
        
        # 3. Corridor Bonus
        corridor_score = self.corridor_map.get(nxt, 0)
        if corridor_score >= 6:
            move_cost *= settings.CORRIDOR_BONUS
        elif corridor_score <= 2:
            move_cost *= 1.3
        
        # 4. Highway Bonus (จาก RouteAnalyzer)
        if use_route_system and self.route_analyzer:
            highway_bonus = self.route_analyzer.get_highway_bonus(nxt)
            if highway_bonus > 0:
                move_cost *= max(0.85, 1.0 - highway_bonus * 0.03)
            
            if self.route_analyzer.is_on_main_corridor(nxt):
                move_cost *= 0.92
        
        # 5. Momentum Bonus
        if not GridUtils.is_turn(last_dir, new_dir) and robot["momentum"] > 0:
            move_cost *= max(0.65, 1.0 - robot["momentum"] * 0.06)
        
        # 6. Narrow Passage Detection
        if self._is_narrow_passage(nxt):
            priority = self._get_robot_priority(robot)
            if priority < 2000:
                move_cost *= 1.5
        
        return move_cost
    
    def _is_narrow_passage(self, pos):
        """ตรวจสอบว่าตำแหน่งนี้เป็นทางแคบหรือไม่"""
        r, c = pos
        open_count = 0
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if GridUtils.in_bounds(nr, nc) and (nr, nc) not in self.obstacles:
                open_count += 1
        return open_count <= 2
    
    def _get_robot_priority(self, robot):
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
    
    def _fallback_astar(self, start, goal, robot, blocked):
        """Fallback A* แบบเดิม (ไม่มี time dimension)"""
        if start == goal:
            return []
        
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
            
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = current[0] + dr, current[1] + dc
                nxt = (nr, nc)
                new_dir = (dr, dc)
                
                if not GridUtils.in_bounds(nr, nc) or nxt in self.obstacles or nxt in blocked:
                    continue
                if nxt != goal and not self.can_enter_dropoff(robot, nxt):
                    continue
                if nxt != goal and not self.can_enter_pickup(robot, nxt):
                    continue
                
                move_cost = self._calculate_move_cost(robot, current, nxt, last_dir, new_dir, False)
                new_g = g + move_cost
                new_state = (nxt, new_dir)
                
                if new_state not in g_score or new_g < g_score[new_state]:
                    g_score[new_state] = new_g
                    h = GridUtils.manhattan(nxt, goal)
                    f = new_g + h
                    new_path = path + [nxt]
                    heapq.heappush(open_set, (f, new_g, nxt, new_dir, new_path))
        
        return []
    
    def smooth_path(self, path, robot):
        """
        ปิดการ smooth path เพื่อให้ robot เดินทีละ node เท่านั้น
        """
        return path
