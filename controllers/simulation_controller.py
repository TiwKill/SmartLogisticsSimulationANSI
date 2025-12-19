"""
Simulation Controller (Refactored)
ควบคุมการทำงานหลักของ Smart Logistics Simulation
ใช้ Composition pattern กับ modules ที่แยกออกมา
"""

import os
import json
import joblib

from loguru import logger

from core.settings import settings
from utils.grid_utils import GridUtils
from utils.display_manager import DisplayManager, SimulationRenderer
from utils.pathfinding import PathFinder
from utils.deadlock_resolver import DeadlockResolver
from utils.robot_manager import RobotManager


class SimulationController:
    def __init__(self, config_path):
        self.obstacles = set()
        self.corridor_map = {}
        self.robots = []
        self.packages = {}
        self.config_data = self._load_config(config_path)
        
        # Load settings from config if available
        self._load_settings_from_config()
        
        # Display Manager for activity tracking
        self.display = DisplayManager()
        
        # Initialize Logic
        self._init_obstacles()
        self._compute_corridor_map()
        self._init_robots()
        self._init_packages()
        self._validate_config()

        self.logger = logger

        # Load deadlock model
        self.deadlock_model = joblib.load(settings.DEADLOCK_MODEL_PATH)

        # Initialize modules
        self._init_modules()
        
        # Renderer for display
        self.renderer = SimulationRenderer(self.display)

    def _init_modules(self):
        """Initialize all sub-modules"""
        self.pathfinder = PathFinder(
            self.obstacles, 
            self.corridor_map, 
            self.robots, 
            self.packages,
            self.deadlock_model
        )
        
        self.deadlock_resolver = DeadlockResolver(
            self.obstacles,
            self.corridor_map,
            self.robots,
            self.packages
        )
        
        self.robot_manager = RobotManager(
            self.robots,
            self.packages,
            self.obstacles,
            self.corridor_map,
            self.pathfinder
        )

    def _load_settings_from_config(self):
        """โหลด settings จาก JSON config"""
        config_settings = self.config_data.get('settings', {})
        if 'rows' in config_settings:
            settings.ROWS = config_settings['rows']
        if 'cols' in config_settings:
            settings.COLS = config_settings['cols']
        if 'sleep' in config_settings:
            settings.SLEEP = config_settings['sleep']
        if 'max_wait' in config_settings:
            settings.MAX_WAIT = config_settings['max_wait']
        if 'max_steps' in config_settings:
            settings.MAX_STEPS = config_settings['max_steps']

    def _load_config(self, path):
        """อ่านไฟล์ JSON Config"""
        if not os.path.exists(path):
            print(f"Error: Config file '{path}' not found.")
            exit(1)
            
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _init_obstacles(self):
        for wall in self.config_data.get('walls', []):
            self.obstacles.update(GridUtils.create_wall(wall))

    def _compute_corridor_map(self):
        for r in range(settings.ROWS):
            for c in range(settings.COLS):
                if (r, c) in self.obstacles:
                    self.corridor_map[(r, c)] = 0
                    continue
                
                open_neighbors = 0
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if GridUtils.in_bounds(nr, nc) and (nr, nc) not in self.obstacles:
                            open_neighbors += 1
                
                self.corridor_map[(r, c)] = open_neighbors

    def _init_robots(self):
        """โหลดข้อมูล Robot รองรับเฉพาะ Dictionary Format"""
        for i, config in enumerate(self.config_data.get('robots', [])):
            if not isinstance(config, dict):
                print(f"Skipping invalid robot config (must be dict): {config}")
                continue

            robot_id = config.get("id", i + 1)
            name = config.get("name", f"R{robot_id}")
            pos_input = config.get("pos")
            
            if pos_input is None:
                print(f"Skipping robot {name}: missing 'pos'")
                continue

            pos = GridUtils.parse_pos(pos_input)
            
            self.robots.append({
                "id": robot_id,
                "name": name,
                "pos": pos,
                "home": pos,
                "state": "IDLE",
                "path": [],
                "package": None,
                "wait_count": 0,
                "evac_target": None,
                "last_dir": (0, 0),
                "momentum": 0,
                "total_turns": 0,
                "decision_mode": "NORMAL",
                "yield_to": None,
                "stuck_at": None,
                "stuck_count": 0,
                "last_decision_step": 0,
                "failed_paths": set(),
                "position_history": [],
                "evac_start_step": 0,
                "yield_start_step": 0,
            })

    def _init_packages(self):
        """โหลดข้อมูล Package รองรับเฉพาะ Dictionary Format"""
        for i, config in enumerate(self.config_data.get('packages', [])):
            if not isinstance(config, dict):
                continue

            name = config.get("name", f"P{i+1}")
            pickup = GridUtils.parse_pos(config.get("pickup"))
            dropoff = GridUtils.parse_pos(config.get("dropoff"))

            self.packages[i] = {
                "name": name,
                "pickup": pickup,
                "dropoff": dropoff,
                "status": "WAITING",
                "assigned_to": None
            }

    def _validate_config(self):
        errors = []
        for config in self.config_data.get('robots', []):
            if isinstance(config, dict):
                name = config.get("name", "Unknown")
                pos_input = config.get("pos")
            else:
                name, pos_input = config
                
            pos = GridUtils.parse_pos(pos_input)
            if not GridUtils.in_bounds(*pos):
                errors.append(f"Robot {name}: position {pos} out of bounds")
            if pos in self.obstacles:
                errors.append(f"Robot {name}: position {pos} is inside a wall")
        
        for config in self.config_data.get('packages', []):
            if isinstance(config, dict):
                name = config.get("name", "Unknown")
                pickup_input = config.get("pickup")
                dropoff_input = config.get("dropoff")
            else:
                name, pickup_input, dropoff_input = config
                
            pickup = GridUtils.parse_pos(pickup_input)
            dropoff = GridUtils.parse_pos(dropoff_input)
            
            if not GridUtils.in_bounds(*pickup):
                errors.append(f"Package {name}: pickup {pickup} out of bounds")
            if pickup in self.obstacles:
                errors.append(f"Package {name}: pickup {pickup} is inside a wall")
            
            if not GridUtils.in_bounds(*dropoff):
                errors.append(f"Package {name}: dropoff {dropoff} out of bounds")
            if dropoff in self.obstacles:
                errors.append(f"Package {name}: dropoff {dropoff} is inside a wall")
        
        if errors:
            print("Configuration Errors:")
            for err in errors:
                print(f"  - {err}")
            print("\nPlease fix the configuration before running.")
            exit(1)

    # ======================
    # DELEGATOR METHODS
    # ======================
    
    # PathFinder delegators
    def predict_future_positions(self, robot, steps=3):
        return self.pathfinder.predict_future_positions(robot, steps)

    def get_dynamic_traffic_cost(self, pos, robot, future_predictions):
        return self.pathfinder.get_dynamic_traffic_cost(pos, robot, future_predictions)

    def build_deadlock_features(self, robot, curr, nxt):
        return self.pathfinder.build_deadlock_features(robot, curr, nxt)

    def smart_astar(self, start, goal, blocked, robot):
        return self.pathfinder.smart_astar(start, goal, blocked, robot)

    def smooth_path(self, path, robot):
        return self.pathfinder.smooth_path(path, robot)

    def is_narrow_passage(self, pos):
        return self.pathfinder.is_narrow_passage(pos)

    # DeadlockResolver delegators
    def get_robot_by_id(self, robot_id):
        return self.deadlock_resolver.get_robot_by_id(robot_id)

    def get_robot_importance(self, robot):
        return self.deadlock_resolver.get_robot_importance(robot)

    def get_emergency_move(self, robot):
        return self.deadlock_resolver.get_emergency_move(robot)

    def trace_wait_chain(self, start_robot, max_depth=10):
        return self.deadlock_resolver.trace_wait_chain(start_robot, max_depth)

    def detect_deadlock_group(self):
        return self.deadlock_resolver.detect_deadlock_group()

    def resolve_deadlock_group(self, group):
        return self.deadlock_resolver.resolve_deadlock_group(group)

    def decide_who_yields(self, robot1, robot2):
        return self.deadlock_resolver.decide_who_yields(robot1, robot2)

    def find_yield_position(self, robot, yield_to_robot):
        return self.deadlock_resolver.find_yield_position(robot, yield_to_robot)

    def find_retreat_path(self, robot, steps=3):
        return self.deadlock_resolver.find_retreat_path(robot, steps)

    def make_decisive_action(self, robot):
        return self.deadlock_resolver.make_decisive_action(robot)

    def get_critical_paths(self):
        return self.deadlock_resolver.get_critical_paths()

    def is_in_critical_path(self, robot, critical_paths):
        return self.deadlock_resolver.is_in_critical_path(robot, critical_paths)

    def find_evacuation_spot(self, robot, critical_paths, reserved):
        return self.deadlock_resolver.find_evacuation_spot(robot, critical_paths, reserved)

    def is_near_active_dropoff(self, robot, radius=3):
        return self.deadlock_resolver.is_near_active_dropoff(robot, radius)

    # RobotManager delegators
    def get_traffic_density(self, pos, robot_id):
        return self.robot_manager.get_traffic_density(pos, robot_id)

    def request_package(self, robot):
        return self.robot_manager.request_package(robot)

    def detect_oscillation(self, robot, window=5):
        return self.robot_manager.detect_oscillation(robot, window)

    def clear_oscillation_history(self, robot):
        return self.robot_manager.clear_oscillation_history(robot)

    def cleanup_orphaned_assignments(self):
        return self.robot_manager.cleanup_orphaned_assignments()

    def reassign_stuck_packages(self):
        return self.robot_manager.reassign_stuck_packages()

    def force_idle_robots_to_work(self):
        return self.robot_manager.force_idle_robots_to_work()

    def fix_robot_states(self):
        return self.robot_manager.fix_robot_states()

    def force_reset_stuck_state(self, robot, current_step):
        return self.robot_manager.force_reset_stuck_state(robot, current_step)

    def get_blocked_for_robot(self, robot, reserved_positions):
        return self.robot_manager.get_blocked_for_robot(robot, reserved_positions)

    # ======================
    # HELPER & RULES (kept in controller)
    # ======================
    def is_safe_cell(self, pos):
        r, c = pos
        if not GridUtils.in_bounds(r, c):
            return False
        if pos in self.obstacles:
            return False
        return True

    def can_enter_dropoff(self, robot, pos):
        return self.pathfinder.can_enter_dropoff(robot, pos)

    def can_enter_pickup(self, robot, pos):
        return self.pathfinder.can_enter_pickup(robot, pos)

    def is_collision_free(self, robot, pos):
        for other in self.robots:
            if other["id"] != robot["id"] and other["pos"] == pos:
                return False
        return True

    def is_valid_move(self, robot, pos, reserved_positions=None):
        if not self.is_safe_cell(pos):
            return False
        if not self.is_collision_free(robot, pos):
            return False
        if not self.can_enter_dropoff(robot, pos):
            return False
        if not self.can_enter_pickup(robot, pos):
            return False
        if reserved_positions and pos in reserved_positions:
            return False
        return True

    def get_robot_priority(self, robot):
        return self.pathfinder.get_robot_priority(robot)

    def is_swap(self, rb, nxt, planned_moves):
        for oid, onxt in planned_moves.items():
            if oid == rb["id"]: continue
            other = self.get_robot_by_id(oid)
            if other and onxt == rb["pos"] and nxt == other["pos"]:
                return True
        return False

    # ======================
    # RENDER (delegated)
    # ======================
    def render(self, step):
        self.renderer.render(step, self.robots, self.packages, self.obstacles, self.corridor_map)

    def render_final_statistics(self, total_steps):
        self.renderer.render_final_statistics(total_steps, self.robots, self.packages)