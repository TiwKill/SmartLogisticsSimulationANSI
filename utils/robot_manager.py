"""
Robot Manager Module
จัดการ Robot และ Package assignments สำหรับ Smart Logistics Simulation
"""

from core.settings import settings
from utils.grid_utils import GridUtils


class RobotManager:
    """จัดการ Robot และ Package assignments"""
    
    def __init__(self, robots, packages, obstacles, corridor_map, pathfinder):
        self.robots = robots
        self.packages = packages
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.pathfinder = pathfinder
    
    def get_robot_by_id(self, robot_id):
        """Helper method to find a robot by its ID"""
        for rb in self.robots:
            if rb["id"] == robot_id:
                return rb
        return None

    def get_traffic_density(self, pos, robot_id):
        """คำนวณความหนาแน่นของ traffic รอบตำแหน่ง"""
        density = 0
        for rb in self.robots:
            if rb["id"] == robot_id:
                continue
            dist = GridUtils.manhattan(pos, rb["pos"])
            if dist == 0:
                density += 10
            elif dist <= 2:
                density += 5 / dist
            elif dist <= 4:
                density += 2 / dist
        return density

    def is_narrow_passage(self, pos):
        """ตรวจสอบว่าตำแหน่งนี้เป็นทางแคบหรือไม่"""
        r, c = pos
        open_count = 0
        
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if GridUtils.in_bounds(nr, nc) and (nr, nc) not in self.obstacles:
                open_count += 1
        
        return open_count <= 2

    def request_package(self, robot):
        """ขอ package ใหม่สำหรับ robot"""
        candidates = []
        for pid, pkg in self.packages.items():
            if pkg["status"] == "WAITING" and pkg["assigned_to"] is None:
                pickup_dist = GridUtils.manhattan(robot["pos"], pkg["pickup"])
                dropoff_dist = GridUtils.manhattan(pkg["pickup"], pkg["dropoff"])
                
                traffic_cost = self.get_traffic_density(pkg["pickup"], robot["id"])
                passage_penalty = 2.0 if self.is_narrow_passage(pkg["pickup"]) else 0.0
                
                competing_robots = sum(
                    1 for rb in self.robots 
                    if rb["id"] != robot["id"] 
                    and rb["package"] is not None 
                    and GridUtils.manhattan(rb["pos"], pkg["pickup"]) < pickup_dist
                )
                
                total_cost = (
                    pickup_dist * 1.0 + 
                    dropoff_dist * 0.2 + 
                    traffic_cost * 1.5 +
                    passage_penalty +
                    competing_robots * 3.0
                )
                
                candidates.append((total_cost, pid))
        
        if candidates:
            candidates.sort()
            best_pid = candidates[0][1]
            self.packages[best_pid]["assigned_to"] = robot["id"]
            return best_pid
        return None

    def detect_oscillation(self, robot, window=5):
        """ตรวจจับว่า robot เดินวนซ้ำหรือไม่"""
        if 'position_history' not in robot:
            robot['position_history'] = []
        
        robot['position_history'].append(robot['pos'])
        
        if len(robot['position_history']) > 10:
            robot['position_history'].pop(0)
        
        if len(robot['position_history']) >= window:
            recent = robot['position_history'][-window:]
            unique_positions = len(set(recent))
            
            if unique_positions <= 3:
                return True
        
        return False

    def clear_oscillation_history(self, robot):
        """ล้างประวัติการเดิน"""
        if 'position_history' in robot:
            robot['position_history'] = []

    def cleanup_orphaned_assignments(self):
        """ล้าง package assignments ที่ไม่มี robot ทำงานอยู่"""
        for pid, pkg in self.packages.items():
            if pkg["status"] != "WAITING": continue
            if pkg["assigned_to"] is None: continue
            
            assigned_robot = self.get_robot_by_id(pkg["assigned_to"])
            if assigned_robot is None: continue
            robot_is_working_on_this = (
                assigned_robot["package"] == pid and 
                assigned_robot["state"] in ["TO_PICKUP", "TO_DROPOFF"]
            )
            if not robot_is_working_on_this:
                pkg["assigned_to"] = None

    def reassign_stuck_packages(self):
        """Reassign packages จาก robot ที่ติดค้าง"""
        for pid, pkg in self.packages.items():
            if pkg["status"] != "WAITING" or pkg["assigned_to"] is None: continue
            
            assigned_robot = self.get_robot_by_id(pkg["assigned_to"])
            if assigned_robot is None: continue
            if assigned_robot["wait_count"] > settings.REASSIGN_THRESHOLD:
                best_robot = None
                best_dist = GridUtils.manhattan(assigned_robot["pos"], pkg["pickup"])
                
                for rb in self.robots:
                    if rb["id"] == assigned_robot["id"]: continue
                    if rb["state"] != "IDLE" and rb["state"] != "HOME": continue
                    if rb["wait_count"] > settings.YIELD_THRESHOLD: continue
                    
                    dist = GridUtils.manhattan(rb["pos"], pkg["pickup"])
                    if dist < best_dist:
                        best_dist = dist
                        best_robot = rb
                
                if best_robot:
                    assigned_robot["package"] = None
                    assigned_robot["state"] = "IDLE"
                    assigned_robot["path"] = []
                    assigned_robot["failed_paths"].clear()
                    
                    pkg["assigned_to"] = best_robot["id"]
                    best_robot["package"] = pid
                    blocked = self.get_blocked_for_robot(best_robot, set())
                    best_robot["path"] = self.pathfinder.smart_astar(best_robot["pos"], pkg["pickup"], blocked, best_robot)
                    best_robot["state"] = "TO_PICKUP"
                    best_robot["decision_mode"] = "NORMAL"
                    best_robot["failed_paths"].clear()
                    best_robot["wait_count"] = 0

    def force_idle_robots_to_work(self):
        """บังคับให้ robot ที่ว่างไปรับงาน"""
        for rb in self.robots:
            if rb["state"] != "IDLE": continue
            if rb["package"] is not None: continue
            
            rb["failed_paths"].clear()
            best_pid = None
            best_cost = float('inf')
            
            for pid, pkg in self.packages.items():
                if pkg["status"] != "WAITING": continue
                if pkg["assigned_to"] is not None: continue
                
                pickup_dist = GridUtils.manhattan(rb["pos"], pkg["pickup"])
                if pickup_dist < best_cost:
                    best_cost = pickup_dist
                    best_pid = pid
            
            if best_pid is not None:
                self.packages[best_pid]["assigned_to"] = rb["id"]
                rb["package"] = best_pid
                blocked = self.get_blocked_for_robot(rb, set())
                rb["path"] = self.pathfinder.smart_astar(rb["pos"], self.packages[best_pid]["pickup"], blocked, rb)
                rb["state"] = "TO_PICKUP"
                rb["decision_mode"] = "NORMAL"
                rb["failed_paths"].clear()
                rb["wait_count"] = 0

    def fix_robot_states(self):
        """แก้ไข state ของ robot ที่ผิดปกติ"""
        for rb in self.robots:
            if rb["package"] is not None:
                pkg = self.packages[rb["package"]]
                if pkg["status"] == "PICKED" and rb["state"] == "IDLE":
                    print(f"[FIX] {rb['name']} has package {pkg['name']} but was IDLE, setting to TO_DROPOFF")
                    rb["state"] = "TO_DROPOFF"
                    rb["failed_paths"].clear()
                    blocked = self.get_blocked_for_robot(rb, set())
                    rb["path"] = self.pathfinder.smart_astar(rb["pos"], pkg["dropoff"], blocked, rb)
                    rb["wait_count"] = 0
                elif pkg["status"] == "WAITING" and rb["state"] == "IDLE":
                    print(f"[FIX] {rb['name']} assigned {pkg['name']} but was IDLE, setting to TO_PICKUP")
                    rb["state"] = "TO_PICKUP"
                    rb["failed_paths"].clear()
                    blocked = self.get_blocked_for_robot(rb, set())
                    rb["path"] = self.pathfinder.smart_astar(rb["pos"], pkg["pickup"], blocked, rb)
                    rb["wait_count"] = 0

    def force_reset_stuck_state(self, robot, current_step):
        """บังคับ reset state ของ robot ที่ติดค้าง"""
        print(f"[FORCE RESET] {robot['name']} stuck in {robot['state']}/{robot['decision_mode']} - Resetting to IDLE")
        
        robot["state"] = "IDLE"
        robot["decision_mode"] = "NORMAL"
        robot["path"] = []
        robot["evac_target"] = None
        robot["yield_to"] = None
        robot["wait_count"] = 0
        robot["failed_paths"].clear()
        robot["position_history"] = []
        robot["evac_start_step"] = 0
        robot["yield_start_step"] = 0
        robot["momentum"] = 0
        
        if robot["package"] is not None:
            pkg = self.packages[robot["package"]]
            if pkg["status"] == "WAITING":
                pkg["assigned_to"] = None
                robot["package"] = None

    def get_blocked_for_robot(self, robot, reserved_positions):
        """หาตำแหน่งที่ blocked สำหรับ robot"""
        blocked = set(self.obstacles)
        for other in self.robots:
            if other["id"] != robot["id"]:
                blocked.add(other["pos"])
        blocked.update(reserved_positions)
        blocked.discard(robot["pos"])
        
        my_dropoff = None
        if robot["package"] is not None and robot["state"] == "TO_DROPOFF":
            my_dropoff = self.packages[robot["package"]]["dropoff"]
        
        for pid, pkg in self.packages.items():
            if pkg["status"] == "PICKED" and pkg["dropoff"] != my_dropoff:
                blocked.add(pkg["dropoff"])
        return blocked
