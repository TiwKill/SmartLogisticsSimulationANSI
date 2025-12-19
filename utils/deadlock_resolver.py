"""
Deadlock Resolver Module
จัดการการตรวจจับและแก้ไข Deadlock สำหรับ Smart Logistics Simulation
"""

import random
from core.settings import settings
from utils.grid_utils import GridUtils


class DeadlockResolver:
    """จัดการการตรวจจับและแก้ไข Deadlock"""
    
    def __init__(self, obstacles, corridor_map, robots, packages):
        self.obstacles = obstacles
        self.corridor_map = corridor_map
        self.robots = robots
        self.packages = packages
    
    def get_robot_by_id(self, robot_id):
        """Helper method to find a robot by its ID"""
        for rb in self.robots:
            if rb["id"] == robot_id:
                return rb
        return None

    def get_robot_importance(self, robot):
        """คำนวณความสำคัญของ robot"""
        score = 0
        if robot["state"] == "TO_DROPOFF":
            score += 1000
            if robot["package"] is not None and robot["path"]:
                score += 500 - min(len(robot["path"]), 500)
        elif robot["state"] == "TO_PICKUP":
            score += 500
        elif robot["state"] == "HOME":
            score += 100
        elif robot["state"] == "EVACUATING":
            score += 50
        
        score += robot["momentum"] * 20
        score += robot["wait_count"] * 10
        return score

    def is_safe_cell(self, pos):
        """ตรวจสอบว่าตำแหน่งปลอดภัยหรือไม่"""
        r, c = pos
        if not GridUtils.in_bounds(r, c):
            return False
        if pos in self.obstacles:
            return False
        return True

    def get_emergency_move(self, robot):
        """หาตำแหน่งฉุกเฉินสำหรับ robot"""
        directions = [(-1,0), (1,0), (0,-1), (0,1)]
        random.shuffle(directions)
        
        for dr, dc in directions:
            nr, nc = robot["pos"][0] + dr, robot["pos"][1] + dc
            nxt = (nr, nc)
            
            if not self.is_safe_cell(nxt): continue
            if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]): continue
            if nxt in robot["failed_paths"]: continue
            return nxt
        
        robot["failed_paths"].clear()
        for dr, dc in directions:
            nr, nc = robot["pos"][0] + dr, robot["pos"][1] + dc
            nxt = (nr, nc)
            if not self.is_safe_cell(nxt): continue
            if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]): continue
            return nxt
        return None

    def trace_wait_chain(self, start_robot, max_depth=10):
        """ติดตาม chain ของ robots ที่รอกัน"""
        chain = [start_robot["id"]]
        current = start_robot
        for _ in range(max_depth):
            if not current["path"]: break
            next_pos = current["path"][0]
            next_robot = None
            for rb in self.robots:
                if rb["id"] != current["id"] and rb["pos"] == next_pos:
                    next_robot = rb
                    break
            if next_robot is None: break
            if next_robot["id"] in chain:
                chain.append(next_robot["id"])
                break
            chain.append(next_robot["id"])
            current = next_robot
        return chain

    def detect_deadlock_group(self):
        """ตรวจจับกลุ่ม robots ที่เกิด deadlock"""
        deadlock_groups = []
        waiting_robots = [rb for rb in self.robots if rb["wait_count"] > settings.DECISION_WAIT_THRESHOLD]
        if len(waiting_robots) < 2: return deadlock_groups
        
        visited = set()
        for rb in waiting_robots:
            if rb["id"] in visited: continue
            group = [rb["id"]]
            visited.add(rb["id"])
            
            if rb["path"]:
                next_pos = rb["path"][0]
                for other in waiting_robots:
                    if other["id"] != rb["id"] and other["pos"] == next_pos:
                        group.append(other["id"])
                        visited.add(other["id"])
                        if other["path"] and other["path"][0] == rb["pos"]:
                            deadlock_groups.append(group)
                            break
            
            if len(group) >= 2:
                chain = self.trace_wait_chain(rb)
                if len(chain) >= 3 and chain[0] == chain[-1]:
                    deadlock_groups.append(list(set(chain[:-1])))
        return deadlock_groups

    def resolve_deadlock_group(self, group):
        """แก้ไข deadlock group"""
        if len(group) < 2: return
        min_importance = float('inf')
        least_important_robot = None
        for rid in group:
            rb = self.get_robot_by_id(rid)
            if rb is None: continue
            imp = self.get_robot_importance(rb)
            if imp < min_importance:
                min_importance = imp
                least_important_robot = rb
        
        if least_important_robot:
            emergency_pos = self.get_emergency_move(least_important_robot)
            if emergency_pos:
                least_important_robot["path"] = [emergency_pos]
                least_important_robot["decision_mode"] = "FORCED"
                least_important_robot["wait_count"] = 0
                print(f"[DEADLOCK] {least_important_robot['name']} forced to move to {GridUtils.pos_to_str(emergency_pos)}")

    def decide_who_yields(self, robot1, robot2):
        """ตัดสินใจว่า robot ตัวไหนควรหลบ"""
        imp1 = self.get_robot_importance(robot1)
        imp2 = self.get_robot_importance(robot2)
        if abs(imp1 - imp2) > 100:
            return robot1 if imp1 < imp2 else robot2
        path1_len = len(robot1["path"]) if robot1["path"] else 999
        path2_len = len(robot2["path"]) if robot2["path"] else 999
        if path1_len != path2_len:
            return robot1 if path1_len > path2_len else robot2
        return robot1 if robot1["id"] < robot2["id"] else robot2

    def find_yield_position(self, robot, yield_to_robot):
        """หาตำแหน่งสำหรับหลบให้ robot อื่น"""
        other_path = set(yield_to_robot["path"][:5]) if yield_to_robot["path"] else set()
        other_path.add(yield_to_robot["pos"])
        best_pos = None
        best_score = -999
        
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nr, nc = robot["pos"][0] + dr, robot["pos"][1] + dc
            nxt = (nr, nc)
            if not self.is_safe_cell(nxt): continue
            if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]): continue
            if nxt in other_path: continue
            
            score = self.corridor_map.get(nxt, 0)
            min_dist_to_path = min((GridUtils.manhattan(nxt, p) for p in other_path), default=0)
            score += min_dist_to_path * 2
            
            if score > best_score:
                best_score = score
                best_pos = nxt
        return best_pos

    def find_retreat_path(self, robot, steps=3):
        """หา path สำหรับถอยหลัง"""
        opposite_dir = (-robot["last_dir"][0], -robot["last_dir"][1])
        retreat_positions = []
        current = robot["pos"]
        for _ in range(steps):
            nr = current[0] + opposite_dir[0]
            nc = current[1] + opposite_dir[1]
            nxt = (nr, nc)
            if not self.is_safe_cell(nxt): break
            if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]): break
            retreat_positions.append(nxt)
            current = nxt
        return retreat_positions

    def make_decisive_action(self, robot):
        """ตัดสินใจการกระทำเมื่อ robot ติดค้าง"""
        wait = robot["wait_count"]
        
        if robot["stuck_at"] == robot["pos"]:
            robot["stuck_count"] += 1
        else:
            robot["stuck_at"] = robot["pos"]
            robot["stuck_count"] = 1
        
        if wait >= 3 and robot["package"] is not None:
            robot["failed_paths"].clear()
        
        if wait < settings.YIELD_THRESHOLD:
            return ("WAIT", None)
        
        blocking_robot = None
        if robot["path"]:
            next_pos = robot["path"][0]
            for other in self.robots:
                if other["id"] != robot["id"] and other["pos"] == next_pos:
                    blocking_robot = other
                    break
        
        if settings.YIELD_THRESHOLD <= wait < settings.DECISION_WAIT_THRESHOLD:
            if blocking_robot:
                yielder = self.decide_who_yields(robot, blocking_robot)
                if yielder["id"] == robot["id"]:
                    yield_pos = self.find_yield_position(robot, blocking_robot)
                    if yield_pos:
                        robot["decision_mode"] = "YIELDING"
                        robot["yield_to"] = blocking_robot["id"]
                        return ("YIELD", yield_pos)
            
            if not robot["path"]:
                robot["failed_paths"].clear()
                return ("REPATH", None)
            return ("WAIT", None)
        
        if settings.DECISION_WAIT_THRESHOLD <= wait < settings.FORCE_MOVE_THRESHOLD:
            robot["failed_paths"].clear()
            if robot["path"]:
                robot["failed_paths"].add(robot["path"][0])
            robot["decision_mode"] = "NORMAL"
            return ("REPATH", None)
        
        if settings.FORCE_MOVE_THRESHOLD <= wait < settings.DEADLOCK_THRESHOLD:
            robot["failed_paths"].clear()
            retreat_path = self.find_retreat_path(robot)
            if retreat_path:
                robot["decision_mode"] = "RETREAT"
                return ("RETREAT", retreat_path)
            emergency_pos = self.get_emergency_move(robot)
            if emergency_pos:
                robot["decision_mode"] = "FORCED"
                return ("EMERGENCY", emergency_pos)
            return ("REPATH", None)
        
        if wait >= settings.DEADLOCK_THRESHOLD:
            robot["failed_paths"].clear()
            robot["stuck_count"] = 0
            
            if robot["state"] in ["IDLE", "HOME"]:
                emergency_pos = self.get_emergency_move(robot)
                if emergency_pos:
                    robot["decision_mode"] = "FORCED"
                    return ("EMERGENCY", emergency_pos)
            
            if robot["path"]:
                next_pos = robot["path"][0]
                occupant = None
                for other in self.robots:
                    if other["id"] != robot["id"] and other["pos"] == next_pos:
                        occupant = other
                        break
                if occupant and self.get_robot_importance(robot) > self.get_robot_importance(occupant) + 200:
                    occupant["decision_mode"] = "FORCED"
                    evac_pos = self.find_yield_position(occupant, robot)
                    if evac_pos:
                        occupant["path"] = [evac_pos]
                        occupant["state"] = "EVACUATING"
                        occupant["evac_target"] = evac_pos
                        return ("WAIT", None)
            
            emergency_pos = self.get_emergency_move(robot)
            if emergency_pos:
                robot["decision_mode"] = "FORCED"
                return ("EMERGENCY", emergency_pos)
            return ("REPATH", None)
        
        return ("WAIT", None)

    def get_critical_paths(self):
        """ดึง paths ที่สำคัญของ robots ที่กำลังส่งของ"""
        critical_paths = {}
        for rb in self.robots:
            if rb["state"] == "TO_DROPOFF" and rb["package"] is not None:
                path_set = set(rb["path"])
                critical_paths[rb["id"]] = path_set
        return critical_paths

    def is_in_critical_path(self, robot, critical_paths):
        """ตรวจสอบว่า robot อยู่ใน critical path หรือไม่"""
        for crit_id, path_set in critical_paths.items():
            if robot["id"] != crit_id and robot["pos"] in path_set:
                return crit_id
        return None

    def find_evacuation_spot(self, robot, critical_paths, reserved):
        """หาจุดหลบที่ดีที่สุด"""
        all_critical = set()
        for path_set in critical_paths.values():
            all_critical.update(path_set)
        
        # 1. ลองหาจุดที่ใกล้ที่สุดก่อน
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
            nr, nc = robot["pos"][0] + dr, robot["pos"][1] + dc
            nxt = (nr, nc)
            
            if not self.is_safe_cell(nxt):
                continue
            if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]):
                continue
            if nxt in reserved:
                continue
            
            if nxt not in all_critical:
                corridor_score = self.corridor_map.get(nxt, 0)
                if corridor_score >= 4:
                    return nxt
        
        # 2. BFS หาที่ไกลออกไป
        visited = {robot["pos"]}
        queue = [(robot["pos"], 0)]
        best_spot = None
        best_score = -999
        
        while queue and len(visited) < 30:
            pos, dist = queue.pop(0)
            
            if dist > 4:
                continue
            
            for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]:
                nr, nc = pos[0] + dr, pos[1] + dc
                nxt = (nr, nc)
                
                if nxt in visited:
                    continue
                visited.add(nxt)
                
                if not self.is_safe_cell(nxt):
                    continue
                
                if any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]):
                    queue.append((nxt, dist + 1))
                    continue
                
                if nxt in reserved:
                    queue.append((nxt, dist + 1))
                    continue
                
                if nxt not in all_critical:
                    score = self.corridor_map.get(nxt, 0) * 2
                    score -= dist * 0.5
                    
                    corner_count = sum(
                        1 for dr2, dc2 in [(-1,0), (1,0), (0,-1), (0,1)]
                        if (nxt[0] + dr2, nxt[1] + dc2) in self.obstacles
                    )
                    if corner_count >= 2:
                        score += 5
                    
                    if score > best_score:
                        best_score = score
                        best_spot = nxt
                
                queue.append((nxt, dist + 1))
        
        if best_spot:
            return best_spot
        
        # 4. หาแค่ที่ว่างที่ใกล้ที่สุด
        for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            nxt = (robot["pos"][0] + dr, robot["pos"][1] + dc)
            if self.is_safe_cell(nxt) and nxt not in reserved:
                if not any(other["pos"] == nxt for other in self.robots if other["id"] != robot["id"]):
                    return nxt
        
        return None

    def is_near_active_dropoff(self, robot, radius=3):
        """ตรวจสอบว่า robot อยู่ใกล้จุด dropoff ที่กำลังใช้งานหรือไม่"""
        for rb in self.robots:
            if rb["state"] == "TO_DROPOFF" and rb["package"] is not None:
                dropoff = self.packages[rb["package"]]["dropoff"]
                if GridUtils.manhattan(robot["pos"], dropoff) <= radius:
                    return True
        return False
