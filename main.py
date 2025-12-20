from loguru import logger
from datetime import datetime
import time
import os
from controllers.simulation_controller import SimulationController
from core.settings import settings
from utils.grid_utils import GridUtils

# ======================
# LOGGER SETUP
# ======================
RUN_TIME = datetime.now().strftime("%Y%m%d_%H%M%S")
LOG_DIR = os.path.join(settings.LOG_DIR, RUN_TIME)
os.makedirs(LOG_DIR, exist_ok=True)

# ลบ default logger (stdout)
logger.remove()

# Logger กลาง (optional)
logger.add(
    os.path.join(LOG_DIR, "system.log"),
    level="INFO",
    format="{time:YYYY-MM-DD HH:mm:ss} | {level} | {message}"
)

def main():
    if not os.path.exists(settings.PATTERN_DIR):
        print("Config file not found.")
        return

    print(f"Loading config from: {settings.PATTERN_DIR}")
    sim = SimulationController(settings.PATTERN_DIR)
    step = 0

    robot_loggers = {}
    for rb in sim.robots:
        robot_logger = logger.bind(robot=rb["name"])
        robot_logger.add(
            os.path.join(LOG_DIR, f"{rb['name']}.log"),
            level="INFO",
            format="{time:YYYY-MM-DD HH:mm:ss} | {message}"
        )
        robot_loggers[rb["id"]] = robot_logger
    
    while True:
        step += 1
        
        # Update pathfinder step for Time-Space A*
        sim.update_pathfinder_step(step)
        
        # 1. Maintenance & Cleanup
        sim.fix_robot_states()
        
        if step % settings.ORPHAN_CHECK_INTERVAL == 0:
            sim.cleanup_orphaned_assignments()
        
        if step % settings.IDLE_RECHECK_INTERVAL == 0:
            sim.reassign_stuck_packages()
            sim.force_idle_robots_to_work()
        
        sim.force_idle_robots_to_work()
        
        # 2. Deadlock Detection & Resolution
        deadlock_groups = sim.detect_deadlock_group()
        for group in deadlock_groups:
            sim.resolve_deadlock_group(group)
            sim.display.record_deadlock()
            sim.display.add_activity(f"Deadlock resolved: Group {group}")
        
        # 3. Critical Path & Evacuation Logic
        critical_paths = sim.get_critical_paths()
        
        for rb in sim.robots:
            # 1. ตรวจจับการเดินวนซ้ำ
            if sim.detect_oscillation(rb):
                sim.logger.warning(
                    f"{rb['name']} OSCILLATING at {GridUtils.pos_to_str(rb['pos'])} "
                    f"STATE={rb['state']} MODE={rb['decision_mode']}"
                )
                sim.force_reset_stuck_state(rb, step)
                continue
            
            # 2. Timeout สำหรับ EVACUATING (ไม่ควรเกิน 15 steps)
            if rb["state"] == "EVACUATING":
                if rb["evac_start_step"] == 0:
                    rb["evac_start_step"] = step
                
                evac_duration = step - rb["evac_start_step"]
                
                # ถ้า evac นานเกิน 15 steps หรือถึงจุดหมายแล้ว
                if evac_duration > 15 or rb["pos"] == rb["evac_target"]:
                    sim.logger.warning(
                        f"{rb['name']} EVAC timeout ({evac_duration} steps) - Forcing IDLE"
                    )
                    sim.force_reset_stuck_state(rb, step)
                    continue
            else:
                rb["evac_start_step"] = 0
            
            # 3. Timeout สำหรับ YIELDING mode (ไม่ควนเกิน 10 steps)
            if rb["decision_mode"] == "YIELDING":
                if rb["yield_start_step"] == 0:
                    rb["yield_start_step"] = step
                
                yield_duration = step - rb["yield_start_step"]
                
                if yield_duration > 10:
                    sim.logger.warning(
                        f"{rb['name']} YIELDING timeout ({yield_duration} steps) - Forcing NORMAL"
                    )
                    rb["decision_mode"] = "NORMAL"
                    rb["yield_to"] = None
                    rb["yield_start_step"] = 0
                    rb["path"] = []
                    rb["wait_count"] = 0
            else:
                rb["yield_start_step"] = 0

            if rb["state"] in ["IDLE", "HOME"]:
                blocking_crit = sim.is_in_critical_path(rb, critical_paths)
                near_dropoff = sim.is_near_active_dropoff(rb, radius=2)
                
                # ปรับเงื่อนไขให้เข้มงวดขึ้น - ต้องมีทั้ง blocking_crit และ near_dropoff
                if blocking_crit is not None and near_dropoff:
                    evac_spot = sim.find_evacuation_spot(rb, critical_paths, set())
                    if evac_spot:
                        # ตรวจสอบว่าคุ้มค่าที่จะ evac หรือไม่
                        dist_to_evac = GridUtils.manhattan(rb["pos"], evac_spot)
                        
                        # ถ้าห่างเกิน 3 cells ไม่ต้อง evac
                        if dist_to_evac <= 3:
                            rb["evac_target"] = evac_spot
                            rb["state"] = "EVACUATING"
                            rb["evac_start_step"] = step
                            blocked = sim.get_blocked_for_robot(rb, set())
                            rb["path"] = sim.smart_astar(rb["pos"], evac_spot, blocked, rb)
                            rb["wait_count"] = 0
        
        # 4. Decision Making (Yield/Retreat)
        for rb in sim.robots:
            if rb["wait_count"] >= settings.YIELD_THRESHOLD:
                action_type, action_data = sim.make_decisive_action(rb)
                
                if action_type == "YIELD":
                    robot_loggers[rb["id"]].warning(
                        f"YIELD to R{rb['yield_to']} -> {GridUtils.pos_to_str(action_data)}"
                    )
                    blocked = sim.get_blocked_for_robot(rb, set())
                    rb["path"] = sim.smart_astar(rb["pos"], action_data, blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)
                    if not rb["path"]:
                        rb["path"] = [action_data]
                    rb["evac_target"] = action_data
                    rb["state"] = "EVACUATING"
                    rb["wait_count"] = 0
                    sim.display.record_yield()
                    sim.display.add_activity(f"{rb['name']} YIELD to R{rb['yield_to']}")
                    
                elif action_type == "RETREAT":
                    robot_loggers[rb["id"]].warning(
                        f"RETREAT -> {GridUtils.pos_to_str(action_data)} | "
                        f"POS={GridUtils.pos_to_str(rb['pos'])} | STATE={rb['state']} | WAIT={rb['wait_count']}"
                    )
                    rb["path"] = action_data
                    rb["wait_count"] = 0
                    
                elif action_type == "EMERGENCY":
                    robot_loggers[rb["id"]].error(
                        f"EMERGENCY MOVE -> {GridUtils.pos_to_str(action_data)} | "
                        f"POS={GridUtils.pos_to_str(rb['pos'])} | STATE={rb['state']} | WAIT={rb['wait_count']}"
                    )
                    rb["path"] = [action_data]
                    rb["wait_count"] = 0
                    
                elif action_type == "REPATH":
                    robot_loggers[rb["id"]].warning(
                        f"REPATH | POS={GridUtils.pos_to_str(rb['pos'])} | STATE={rb['state']} | WAIT={rb['wait_count']}"
                    )
                    rb["path"] = []
                    rb["wait_count"] = 0

        # 5. Path Planning & Movement Logic
        sorted_robots = sorted(sim.robots, key=lambda r: sim.get_robot_priority(r), reverse=True)
        reserved_positions = set()
        planned_moves = {}

        for rb in sorted_robots:
            # Planning phase
            if rb["state"] not in ["IDLE", "HOME"] and not rb["path"]:
                target = None
                if rb["state"] == "TO_PICKUP" and rb["package"] is not None:
                    target = sim.packages[rb["package"]]["pickup"]
                elif rb["state"] == "TO_DROPOFF" and rb["package"] is not None:
                    target = sim.packages[rb["package"]]["dropoff"]
                elif rb["state"] == "EVACUATING" and rb["evac_target"]:
                    target = rb["evac_target"]
                
                if target:
                    if rb["wait_count"] > 2:
                        rb["failed_paths"].clear()
                    
                    # เพื่อให้ Robot ตัวรองรู้ว่าเพื่อนตัวก่อนหน้าจองที่ไหนไปแล้ว
                    blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                    blocked.update(rb["failed_paths"])
                    rb["path"] = sim.smart_astar(rb["pos"], target, blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)
                    
                    # Fallback กรณีหาทางไม่ได้
                    if not rb["path"]:
                        blocked_minimal = set(sim.obstacles)
                        for other in sim.robots:
                            if other["id"] != rb["id"]:
                                blocked_minimal.add(other["pos"])
                        rb["path"] = sim.smart_astar(rb["pos"], target, blocked_minimal, rb)
                        if rb["path"] and len(rb["path"]) > 2:
                            rb["path"] = sim.smooth_path(rb["path"], rb)

            # Move prediction
            if rb["path"]:
                nxt = rb["path"][0]
                if sim.is_safe_cell(nxt):
                    planned_moves[rb["id"]] = nxt
                else:
                    planned_moves[rb["id"]] = rb["pos"]
                    rb["wait_count"] += 1
                    rb["momentum"] = 0
            else:
                planned_moves[rb["id"]] = rb["pos"]
                if rb["state"] != "IDLE":
                    rb["wait_count"] += 1
                rb["momentum"] = 0

            # Idle/Home logic
            if rb["state"] == "IDLE":
                rb["failed_paths"].clear()
                
                if rb["package"] is not None:
                    pkg = sim.packages[rb["package"]]
                    if pkg["status"] == "PICKED":
                        rb["state"] = "TO_DROPOFF"
                        blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                        rb["path"] = sim.smart_astar(rb["pos"], pkg["dropoff"], blocked, rb)
                        if rb["path"] and len(rb["path"]) > 2:
                            rb["path"] = sim.smooth_path(rb["path"], rb)
                        rb["wait_count"] = 0
                        continue
                    elif pkg["status"] == "WAITING":
                        rb["state"] = "TO_PICKUP"
                        blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                        rb["path"] = sim.smart_astar(rb["pos"], pkg["pickup"], blocked, rb)
                        if rb["path"] and len(rb["path"]) > 2:
                            rb["path"] = sim.smooth_path(rb["path"], rb)
                        rb["wait_count"] = 0
                        continue
                
                pid = sim.request_package(rb)
                if pid is not None:
                    rb["package"] = pid
                    blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                    rb["path"] = sim.smart_astar(rb["pos"], sim.packages[pid]["pickup"], blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)
                    
                    if not rb["path"]:
                        blocked_minimal = set(sim.obstacles)
                        for other in sim.robots:
                            if other["id"] != rb["id"]:
                                blocked_minimal.add(other["pos"])
                        rb["path"] = sim.smart_astar(rb["pos"], sim.packages[pid]["pickup"], blocked_minimal, rb)
                        if rb["path"] and len(rb["path"]) > 2:
                            rb["path"] = sim.smooth_path(rb["path"], rb)
                    
                    rb["state"] = "TO_PICKUP"
                    rb["decision_mode"] = "NORMAL"
                    rb["failed_paths"].clear()
                    rb["wait_count"] = 0
                elif rb["pos"] != rb["home"]:
                    blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                    rb["path"] = sim.smart_astar(rb["pos"], rb["home"], blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)
                    rb["state"] = "HOME"
                    rb["wait_count"] = 0

            elif rb["state"] == "HOME":
                if rb["pos"] == rb["home"]:
                    rb["state"] = "IDLE"
                    rb["path"] = []
                    rb["decision_mode"] = "NORMAL"
                    rb["failed_paths"].clear()
                    rb["wait_count"] = 0
                elif not rb["path"]:
                    blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                    rb["path"] = sim.smart_astar(rb["pos"], rb["home"], blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)

        # 6. Execute Moves
        for rb in sorted_robots:
            nxt = planned_moves.get(rb["id"], rb["pos"])
            old_pos = rb["pos"]

            if nxt == rb["pos"]:
                reserved_positions.add(rb["pos"])
                rb["wait_count"] += 1
                rb["momentum"] = max(0, rb["momentum"] - 1)
                continue

            if not sim.is_valid_move(rb, nxt, reserved_positions):
                robot_loggers[rb["id"]].warning(
                    f"BLOCKED {GridUtils.pos_to_str(rb['pos'])} -> {GridUtils.pos_to_str(nxt)} | WAIT={rb['wait_count']}"
                )
                rb["wait_count"] += 1
                rb["failed_paths"].add(nxt)
                reserved_positions.add(rb["pos"])
                rb["momentum"] = 0
                
                if sim.is_safe_cell(nxt):
                    if not sim.can_enter_dropoff(rb, nxt) or not sim.can_enter_pickup(rb, nxt):
                        rb["path"] = []
                        robot_loggers[rb["id"]].warning(
                            f"BLOCKED {GridUtils.pos_to_str(rb['pos'])} -> {GridUtils.pos_to_str(nxt)} | WAIT={rb['wait_count']}"
                        )
                        
                continue

            if sim.is_swap(rb, nxt, planned_moves):
                rb["wait_count"] += 1
                reserved_positions.add(rb["pos"])
                rb["momentum"] = 0
                continue

            occupied = any(
                other["pos"] == nxt for other in sim.robots if other["id"] != rb["id"]
            )
            if occupied or nxt in reserved_positions:
                rb["wait_count"] += 1
                reserved_positions.add(rb["pos"])
                rb["momentum"] = max(0, rb["momentum"] - 1)
                continue

            new_dir = GridUtils.get_direction(old_pos, nxt)
            if GridUtils.is_turn(rb["last_dir"], new_dir) and rb["last_dir"] != (0, 0):
                rb["total_turns"] += 1
                rb["momentum"] = max(0, rb["momentum"] - 2)
            else:
                rb["momentum"] = min(5, rb["momentum"] + 1)
            
            rb["last_dir"] = new_dir
            rb["pos"] = nxt
            sim.clear_oscillation_history(rb)
            robot_loggers[rb["id"]].info(
                f"MOVE {GridUtils.pos_to_str(old_pos)} -> {GridUtils.pos_to_str(nxt)} | "
                f"STATE={rb['state']} | MODE={rb['decision_mode']}"
            )
            if rb["path"]: rb["path"].pop(0)
            reserved_positions.add(nxt)
            rb["wait_count"] = 0
            sim.display.record_move()
            sim.display.add_activity(f"{rb['name']} moved {GridUtils.pos_to_str(old_pos)} -> {GridUtils.pos_to_str(nxt)}")
            
            if rb["decision_mode"] not in ["NORMAL", "IDLE"]:
                if rb["state"] != "EVACUATING" or rb["pos"] == rb["evac_target"]:
                    rb["decision_mode"] = "NORMAL"
                    rb["yield_to"] = None

            # Check Task Completion
            if rb["state"] == "TO_PICKUP" and rb["pos"] == sim.packages[rb["package"]]["pickup"]:
                robot_loggers[rb["id"]].info(
                    f"PICKUP {sim.packages[rb['package']]['name']} @ {GridUtils.pos_to_str(rb['pos'])}"
                )
                sim.packages[rb["package"]]["status"] = "PICKED"
                target = sim.packages[rb["package"]]["dropoff"]
                blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                blocked.update(rb["failed_paths"])
                rb["path"] = sim.smart_astar(rb["pos"], target, blocked, rb)
                if rb["path"] and len(rb["path"]) > 2:
                    rb["path"] = sim.smooth_path(rb["path"], rb)
                rb["state"] = "TO_DROPOFF"
                rb["decision_mode"] = "NORMAL"
                rb["failed_paths"].clear()
                sim.display.record_pickup()
                sim.display.add_activity(f"{rb['name']} PICKUP {sim.packages[rb['package']]['name']}")
            
            elif rb["state"] == "TO_DROPOFF" and rb["pos"] == sim.packages[rb["package"]]["dropoff"]:
                robot_loggers[rb["id"]].info(
                    f"DROPOFF {sim.packages[rb['package']]['name']} @ {GridUtils.pos_to_str(rb['pos'])}"
                )
                sim.packages[rb["package"]]["status"] = "DELIVERED"
                sim.packages[rb["package"]]["assigned_to"] = None
                rb["package"] = None
                target = rb["home"]
                blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                blocked.update(rb["failed_paths"])
                rb["path"] = sim.smart_astar(rb["pos"], target, blocked, rb)
                if rb["path"] and len(rb["path"]) > 2:
                    rb["path"] = sim.smooth_path(rb["path"], rb)
                rb["state"] = "HOME"
                rb["decision_mode"] = "NORMAL"
                rb["failed_paths"].clear()
                sim.display.record_dropoff()
                sim.display.add_activity(f"{rb['name']} DROPOFF -> HOME")
            
            elif rb["state"] == "HOME" and rb["pos"] == rb["home"]:
                rb["state"] = "IDLE"
                rb["path"] = []
                rb["decision_mode"] = "NORMAL"
                rb["failed_paths"].clear()
                rb["wait_count"] = 0
                
            elif rb["state"] == "EVACUATING" and rb["pos"] == rb["evac_target"]:
                rb["state"] = "IDLE"
                rb["evac_target"] = None
                rb["path"] = []
                rb["decision_mode"] = "NORMAL"
                rb["yield_to"] = None

        # 7. Final Path Check
        for rb in sim.robots:
            if rb["state"] == "IDLE":
                if rb["package"] is None and rb["pos"] != rb["home"]:
                    blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                    rb["path"] = sim.smart_astar(rb["pos"], rb["home"], blocked, rb)
                    if rb["path"] and len(rb["path"]) > 2:
                        rb["path"] = sim.smooth_path(rb["path"], rb)
                    rb["state"] = "HOME"
            
            elif rb["state"] in ["TO_PICKUP", "TO_DROPOFF", "HOME", "EVACUATING"]:
                if not rb["path"] and rb["wait_count"] > settings.YIELD_THRESHOLD:
                    target = None
                    if rb["state"] == "TO_PICKUP" and rb["package"] is not None:
                        target = sim.packages[rb["package"]]["pickup"]
                    elif rb["state"] == "TO_DROPOFF" and rb["package"] is not None:
                        target = sim.packages[rb["package"]]["dropoff"]
                    elif rb["state"] == "HOME":
                        target = rb["home"]
                    elif rb["state"] == "EVACUATING" and rb["evac_target"]:
                        target = rb["evac_target"]
                    
                    if target:
                        blocked = sim.get_blocked_for_robot(rb, reserved_positions)
                        for fp in rb["failed_paths"]:
                            blocked.add(fp)
                        new_path = sim.smart_astar(rb["pos"], target, blocked, rb)
                        if new_path:
                            rb["path"] = new_path
                            rb["wait_count"] = 0
                            rb["failed_paths"].clear()

        sim.render(step)
        
        # Win Condition
        delivered = sum(1 for p in sim.packages.values() if p["status"] == "DELIVERED")
        if delivered == len(sim.packages):
            all_tasks_complete = True
            for rb in sim.robots:
                if rb["state"] not in ["IDLE", "HOME"]:
                    all_tasks_complete = False
                    break
                if rb["state"] == "HOME" and rb["pos"] != rb["home"]:
                    all_tasks_complete = False
                    break
            
            if all_tasks_complete:
                sim.render_final_statistics(step)
                break
        
        if step > settings.MAX_STEPS:
            print("\n=== MAX STEPS REACHED ===")
            break
        
        time.sleep(settings.SLEEP)

if __name__ == "__main__":
    main()