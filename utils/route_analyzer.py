"""
Route Analyzer Module
=====================

วิเคราะห์ Grid และสร้าง Optimal Routes อัตโนมัติ
- Corridor Detection: หาทางเดินหลักจาก grid layout
- Traffic Flow Analysis: วิเคราะห์กระแสจราจรจาก pickup/dropoff
- Route Generation: สร้าง preferred routes ระหว่าง zones
"""

from collections import defaultdict, deque
from core.settings import settings
from utils.grid_utils import GridUtils


class RouteAnalyzer:
    """วิเคราะห์ Grid และสร้าง Optimal Routes อัตโนมัติ"""
    
    def __init__(self, obstacles, corridor_map, packages):
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.packages = packages
        
        # Route data
        self.main_corridors = set()  # ทางเดินหลัก
        self.highway_map = {}  # แผนที่ทางด่วน (score per cell)
        self.zone_routes = {}  # routes ระหว่าง zones
        
        # Analyze grid
        self._detect_main_corridors()
        self._build_highway_map()
        self._analyze_traffic_zones()
    
    def _detect_main_corridors(self):
        """ตรวจจับทางเดินหลักจาก corridor scores"""
        # หา cells ที่มี corridor score สูง (>=6 = ทางกว้าง)
        for pos, score in self.corridor_map.items():
            if score >= 6:
                self.main_corridors.add(pos)
        
        # เพิ่ม horizontal lanes (rows ที่มี corridor ติดกันยาว)
        for r in range(settings.ROWS):
            consecutive = 0
            start_col = 0
            
            for c in range(settings.COLS):
                pos = (r, c)
                if pos not in self.obstacles and self.corridor_map.get(pos, 0) >= 4:
                    if consecutive == 0:
                        start_col = c
                    consecutive += 1
                else:
                    if consecutive >= 10:  # ทางยาวอย่างน้อย 10 cells
                        for col in range(start_col, start_col + consecutive):
                            self.main_corridors.add((r, col))
                    consecutive = 0
            
            # Check last segment
            if consecutive >= 10:
                for col in range(start_col, start_col + consecutive):
                    self.main_corridors.add((r, col))
        
        # เพิ่ม vertical lanes
        for c in range(settings.COLS):
            consecutive = 0
            start_row = 0
            
            for r in range(settings.ROWS):
                pos = (r, c)
                if pos not in self.obstacles and self.corridor_map.get(pos, 0) >= 4:
                    if consecutive == 0:
                        start_row = r
                    consecutive += 1
                else:
                    if consecutive >= 5:  # ทางยาวอย่างน้อย 5 cells
                        for row in range(start_row, start_row + consecutive):
                            self.main_corridors.add((row, c))
                    consecutive = 0
            
            if consecutive >= 5:
                for row in range(start_row, start_row + consecutive):
                    self.main_corridors.add((row, c))
    
    def _build_highway_map(self):
        """สร้างแผนที่ทางด่วน (highway map) สำหรับ cost calculation"""
        for r in range(settings.ROWS):
            for c in range(settings.COLS):
                pos = (r, c)
                if pos in self.obstacles:
                    self.highway_map[pos] = 0.0
                    continue
                
                score = 0.0
                
                # Bonus สำหรับ main corridor
                if pos in self.main_corridors:
                    score += 3.0
                
                # Bonus สำหรับ corridor score สูง
                corridor_score = self.corridor_map.get(pos, 0)
                score += corridor_score * 0.3
                
                # Bonus สำหรับตำแหน่งใกล้ขอบ (มักเป็นทางเดินหลัก)
                edge_dist = min(r, c, settings.ROWS - 1 - r, settings.COLS - 1 - c)
                if edge_dist <= 3:
                    score += 1.0
                
                self.highway_map[pos] = score
    
    def _analyze_traffic_zones(self):
        """วิเคราะห์ zones จาก pickup/dropoff locations"""
        pickup_positions = []
        dropoff_positions = []
        
        for pid, pkg in self.packages.items():
            pickup_positions.append(pkg["pickup"])
            dropoff_positions.append(pkg["dropoff"])
        
        if not pickup_positions or not dropoff_positions:
            return
        
        # หา center ของแต่ละ zone
        self.pickup_center = self._calculate_center(pickup_positions)
        self.dropoff_center = self._calculate_center(dropoff_positions)
        
        # สร้าง preferred direction
        self._calculate_flow_direction()
    
    def _calculate_center(self, positions):
        """หาจุดศูนย์กลางของกลุ่ม positions"""
        if not positions:
            return (0, 0)
        
        avg_r = sum(p[0] for p in positions) / len(positions)
        avg_c = sum(p[1] for p in positions) / len(positions)
        return (int(avg_r), int(avg_c))
    
    def _calculate_flow_direction(self):
        """คำนวณทิศทางการไหลของ traffic"""
        dr = self.dropoff_center[0] - self.pickup_center[0]
        dc = self.dropoff_center[1] - self.pickup_center[1]
        
        # Normalize
        if dr != 0:
            dr = 1 if dr > 0 else -1
        if dc != 0:
            dc = 1 if dc > 0 else -1
        
        self.flow_direction = (dr, dc)
    
    def get_highway_bonus(self, pos):
        """ดึง highway bonus สำหรับ cost calculation"""
        return self.highway_map.get(pos, 0.0)
    
    def is_on_main_corridor(self, pos):
        """ตรวจสอบว่าตำแหน่งอยู่บน main corridor หรือไม่"""
        return pos in self.main_corridors
    
    def get_preferred_direction(self, from_pos, to_pos, robot_state):
        """หาทิศทางที่ควรเดินตาม traffic flow"""
        if robot_state == "TO_DROPOFF":
            # เดินไป dropoff = ตาม flow direction
            return self.flow_direction if hasattr(self, 'flow_direction') else (0, 0)
        elif robot_state == "TO_PICKUP" or robot_state == "HOME":
            # เดินกลับ = สวน flow direction
            if hasattr(self, 'flow_direction'):
                return (-self.flow_direction[0], -self.flow_direction[1])
        return (0, 0)
    
    def find_optimal_path(self, start, goal, blocked, robot):
        """หา path ที่ optimal โดยใช้ A* กับ highway bonus"""
        import heapq
        
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
            
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            
            # Prefer direction ที่ตรงกับ flow
            preferred = self.get_preferred_direction(current, goal, robot.get("state", "IDLE"))
            if preferred != (0, 0):
                directions.sort(key=lambda d: 0 if d == preferred else (1 if d == last_dir else 2))
            elif last_dir != (0, 0):
                directions.sort(key=lambda d: 0 if d == last_dir else 1)
            
            for dr, dc in directions:
                nr, nc = current[0] + dr, current[1] + dc
                nxt = (nr, nc)
                new_dir = (dr, dc)
                
                if not GridUtils.in_bounds(nr, nc) or nxt in blocked:
                    continue
                
                # Cost calculation
                move_cost = 1.0
                
                # Highway bonus (ลด cost ถ้าอยู่บน highway)
                highway_bonus = self.get_highway_bonus(nxt)
                move_cost *= max(0.5, 1.0 - highway_bonus * 0.1)
                
                # Turn penalty
                if GridUtils.is_turn(last_dir, new_dir) and last_dir != (0, 0):
                    move_cost += 0.8  # ลดจาก 1.5 เพื่อให้เลี้ยวได้ง่ายขึ้น
                
                # Corridor bonus
                corridor_score = self.corridor_map.get(nxt, 0)
                if corridor_score >= 6:
                    move_cost *= 0.7
                
                # Momentum bonus
                if new_dir == last_dir and robot.get("momentum", 0) > 0:
                    move_cost *= 0.85
                
                new_g = g + move_cost
                new_state = (nxt, new_dir)
                
                if new_state not in g_score or new_g < g_score[new_state]:
                    g_score[new_state] = new_g
                    h = GridUtils.manhattan(nxt, goal)
                    
                    # Goal direction bonus
                    goal_dir = (
                        1 if goal[0] > nxt[0] else (-1 if goal[0] < nxt[0] else 0),
                        1 if goal[1] > nxt[1] else (-1 if goal[1] < nxt[1] else 0)
                    )
                    if new_dir[0] == goal_dir[0] or new_dir[1] == goal_dir[1]:
                        h *= 0.9
                    
                    f = new_g + h
                    new_path = path + [nxt]
                    heapq.heappush(open_set, (f, new_g, nxt, new_dir, new_path))
        
        return []


class RouteCache:
    """เก็บ cached routes เพื่อไม่ต้องคำนวณซ้ำ"""
    
    def __init__(self, max_size=1000):
        self.cache = {}
        self.max_size = max_size
        self.access_count = defaultdict(int)
    
    def get(self, start, goal, robot_state):
        """ดึง cached route"""
        key = (start, goal, robot_state)
        if key in self.cache:
            self.access_count[key] += 1
            return self.cache[key].copy()
        return None
    
    def put(self, start, goal, robot_state, path):
        """เก็บ route ใน cache"""
        if len(self.cache) >= self.max_size:
            self._evict_least_used()
        
        key = (start, goal, robot_state)
        self.cache[key] = path.copy()
        self.access_count[key] = 1
    
    def _evict_least_used(self):
        """ลบ routes ที่ใช้น้อยที่สุด"""
        if not self.cache:
            return
        
        # หา key ที่ถูกใช้น้อยที่สุด
        least_used = min(self.access_count, key=self.access_count.get)
        del self.cache[least_used]
        del self.access_count[least_used]
    
    def invalidate(self, positions):
        """ลบ cached routes ที่ผ่าน positions ที่ระบุ"""
        to_remove = []
        positions_set = set(positions)
        
        for key, path in self.cache.items():
            if any(p in positions_set for p in path):
                to_remove.append(key)
        
        for key in to_remove:
            del self.cache[key]
            if key in self.access_count:
                del self.access_count[key]
    
    def clear(self):
        """ล้าง cache ทั้งหมด"""
        self.cache.clear()
        self.access_count.clear()
