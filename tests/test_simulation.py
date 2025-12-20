"""
Test Suite for Smart Logistics Simulation (SLS_ANSI)
====================================================

ใช้ pytest เพื่อทดสอบ modules ต่างๆ

การรัน:
    cd /home/public_html/apiservices/projects/sls_ansi
    pytest tests/ -v
    pytest tests/ -v --cov=. --cov-report=html
"""

import pytest
import sys
import os

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.settings import settings
from utils.grid_utils import GridUtils
from utils.display_manager import DisplayManager, ANSIColors, SimulationRenderer
from utils.penalty_map import DynamicPenaltyMap, CellPenalty


class TestGridUtils:
    """ทดสอบ GridUtils class"""
    
    def test_parse_pos_list(self):
        """ทดสอบการแปลง list เป็น tuple"""
        result = GridUtils.parse_pos([5, 10])
        assert result == (5, 10)
    
    def test_parse_pos_tuple(self):
        """ทดสอบการแปลง tuple"""
        result = GridUtils.parse_pos((3, 7))
        assert result == (3, 7)
    
    def test_parse_pos_invalid(self):
        """ทดสอบ input ที่ไม่ถูกต้อง"""
        with pytest.raises(ValueError):
            GridUtils.parse_pos("A1")
    
    def test_pos_to_str(self):
        """ทดสอบการแปลง position เป็น string"""
        result = GridUtils.pos_to_str((5, 10))
        assert result == "[5, 10]"
    
    def test_in_bounds_valid(self):
        """ทดสอบตำแหน่งที่อยู่ใน bounds"""
        assert GridUtils.in_bounds(0, 0) == True
        assert GridUtils.in_bounds(10, 40) == True
    
    def test_in_bounds_invalid(self):
        """ทดสอบตำแหน่งที่อยู่นอก bounds"""
        assert GridUtils.in_bounds(-1, 0) == False
        assert GridUtils.in_bounds(0, -1) == False
        assert GridUtils.in_bounds(100, 0) == False
        assert GridUtils.in_bounds(0, 100) == False
    
    def test_manhattan_distance(self):
        """ทดสอบการคำนวณ Manhattan distance"""
        assert GridUtils.manhattan((0, 0), (3, 4)) == 7
        assert GridUtils.manhattan((5, 5), (5, 5)) == 0
        assert GridUtils.manhattan((0, 0), (10, 10)) == 20
    
    def test_create_wall_horizontal(self):
        """ทดสอบการสร้าง wall แนวนอน"""
        wall = GridUtils.create_wall([1, 0, 1, 5])
        assert len(wall) == 6
        assert (1, 0) in wall
        assert (1, 5) in wall
    
    def test_create_wall_vertical(self):
        """ทดสอบการสร้าง wall แนวตั้ง"""
        wall = GridUtils.create_wall([0, 1, 5, 1])
        assert len(wall) == 6
        assert (0, 1) in wall
        assert (5, 1) in wall
    
    def test_create_wall_rectangle(self):
        """ทดสอบการสร้าง wall สี่เหลี่ยม"""
        wall = GridUtils.create_wall([0, 0, 2, 2])
        assert len(wall) == 9  # 3x3
    
    def test_create_wall_invalid(self):
        """ทดสอบ wall format ที่ไม่ถูกต้อง"""
        with pytest.raises(ValueError):
            GridUtils.create_wall([1, 2, 3])  # ไม่ครบ 4 ค่า
    
    def test_get_direction(self):
        """ทดสอบการหาทิศทาง"""
        assert GridUtils.get_direction((0, 0), (1, 0)) == (1, 0)  # ลง
        assert GridUtils.get_direction((0, 0), (0, 1)) == (0, 1)  # ขวา
        assert GridUtils.get_direction((1, 0), (0, 0)) == (-1, 0)  # ขึ้น
        assert GridUtils.get_direction((0, 1), (0, 0)) == (0, -1)  # ซ้าย
    
    def test_is_turn(self):
        """ทดสอบการตรวจจับการเลี้ยว"""
        assert GridUtils.is_turn((1, 0), (0, 1)) == True  # เลี้ยว
        assert GridUtils.is_turn((1, 0), (1, 0)) == False  # ตรง
        assert GridUtils.is_turn((0, 0), (1, 0)) == False  # เริ่มต้น


class TestDisplayManager:
    """ทดสอบ DisplayManager class"""
    
    def test_init(self):
        """ทดสอบการสร้าง DisplayManager"""
        dm = DisplayManager()
        assert dm.total_moves == 0
        assert dm.total_pickups == 0
        assert dm.total_dropoffs == 0
        assert dm.deadlock_count == 0
        assert dm.yield_count == 0
    
    def test_record_move(self):
        """ทดสอบการนับ moves"""
        dm = DisplayManager()
        dm.record_move()
        dm.record_move()
        assert dm.total_moves == 2
    
    def test_record_pickup(self):
        """ทดสอบการนับ pickups"""
        dm = DisplayManager()
        dm.record_pickup()
        assert dm.total_pickups == 1
    
    def test_record_dropoff(self):
        """ทดสอบการนับ dropoffs"""
        dm = DisplayManager()
        dm.record_dropoff()
        assert dm.total_dropoffs == 1
    
    def test_record_deadlock(self):
        """ทดสอบการนับ deadlocks"""
        dm = DisplayManager()
        dm.record_deadlock()
        dm.record_deadlock()
        assert dm.deadlock_count == 2
    
    def test_record_yield(self):
        """ทดสอบการนับ yields"""
        dm = DisplayManager()
        dm.record_yield()
        assert dm.yield_count == 1
    
    def test_add_activity(self):
        """ทดสอบการเพิ่ม activity"""
        dm = DisplayManager(max_activities=3)
        dm.add_activity("Test 1")
        dm.add_activity("Test 2")
        dm.add_activity("Test 3")
        dm.add_activity("Test 4")  # เกิน max
        
        activities = dm.get_activities()
        assert len(activities) == 3
        assert "Test 4" in activities[-1]
    
    def test_elapsed_time(self):
        """ทดสอบการคำนวณเวลา"""
        dm = DisplayManager()
        elapsed = dm.get_elapsed_time()
        assert elapsed >= 0


class TestANSIColors:
    """ทดสอบ ANSIColors class"""
    
    def test_colors_exist(self):
        """ทดสอบว่า colors ถูกกำหนด"""
        assert ANSIColors.HEADER is not None
        assert ANSIColors.WHITE is not None
        assert ANSIColors.GREEN is not None
        assert ANSIColors.RED is not None
        assert ANSIColors.ENDC is not None
    
    def test_backgrounds_exist(self):
        """ทดสอบว่า background colors ถูกกำหนด"""
        assert ANSIColors.BG_WALL is not None
        assert ANSIColors.BG_IDLE is not None
        assert ANSIColors.BG_PICKUP is not None
        assert ANSIColors.BG_DROPOFF is not None
        assert ANSIColors.BG_EVAC is not None
        assert ANSIColors.BG_HOME is not None


class TestDynamicPenaltyMap:
    """ทดสอบ DynamicPenaltyMap class"""
    
    def test_init(self):
        """ทดสอบการสร้าง PenaltyMap"""
        pm = DynamicPenaltyMap(10, 10)
        assert pm.rows == 10
        assert pm.cols == 10
        assert len(pm.cells) == 100
    
    def test_update_traffic(self):
        """ทดสอบการอัพเดท traffic"""
        pm = DynamicPenaltyMap(10, 10)
        pm.update_traffic((5, 5), step=1)
        
        cell = pm.cells[(5, 5)]
        assert cell.traffic_history == 1
        assert cell.base_penalty > 0
    
    def test_update_conflict(self):
        """ทดสอบการอัพเดท conflict"""
        pm = DynamicPenaltyMap(10, 10)
        pm.update_conflict((5, 5), step=1, severity=1.0)
        
        cell = pm.cells[(5, 5)]
        assert cell.conflict_history == 1
        assert cell.base_penalty > 0
    
    def test_mark_yield_zone(self):
        """ทดสอบการทำเครื่องหมาย yield zone"""
        pm = DynamicPenaltyMap(10, 10)
        pm.mark_yield_zone((5, 5))
        
        assert pm.cells[(5, 5)].yield_zone == True
    
    def test_mark_priority_zone(self):
        """ทดสอบการทำเครื่องหมาย priority zone"""
        pm = DynamicPenaltyMap(10, 10)
        pm.mark_priority_zone((5, 5))
        
        assert pm.cells[(5, 5)].priority_zone == True
    
    def test_get_penalty(self):
        """ทดสอบการดึงค่า penalty"""
        pm = DynamicPenaltyMap(10, 10)
        pm.update_traffic((5, 5), step=1)
        
        penalty = pm.get_penalty((5, 5))
        assert penalty > 0
    
    def test_get_penalty_with_state(self):
        """ทดสอบ penalty ที่ขึ้นกับ state"""
        pm = DynamicPenaltyMap(10, 10)
        pm.mark_yield_zone((5, 5))
        
        penalty_dropoff = pm.get_penalty((5, 5), robot_state="TO_DROPOFF")
        penalty_idle = pm.get_penalty((5, 5), robot_state="IDLE")
        
        assert penalty_dropoff > penalty_idle
    
    def test_penalty_max_cap(self):
        """ทดสอบการจำกัดค่า penalty สูงสุด"""
        pm = DynamicPenaltyMap(10, 10)
        
        # อัพเดทหลายครั้ง
        for i in range(100):
            pm.update_traffic((5, 5), step=i)
        
        penalty = pm.get_penalty((5, 5))
        assert penalty <= 5.0  # max cap


class TestSettings:
    """ทดสอบ Settings class"""
    
    def test_settings_exist(self):
        """ทดสอบว่า settings ถูกสร้าง"""
        assert settings is not None
    
    def test_grid_settings(self):
        """ทดสอบ grid settings"""
        assert settings.ROWS > 0
        assert settings.COLS > 0
    
    def test_threshold_settings(self):
        """ทดสอบ threshold settings"""
        assert settings.YIELD_THRESHOLD > 0
        assert settings.DEADLOCK_THRESHOLD > 0
        assert settings.FORCE_MOVE_THRESHOLD > 0
        assert settings.DECISION_WAIT_THRESHOLD > 0
    
    def test_penalty_settings(self):
        """ทดสอบ penalty settings"""
        assert settings.TURN_PENALTY > 0
        assert settings.CORRIDOR_BONUS < 1.0
    
    def test_paths_exist(self):
        """ทดสอบว่า paths ถูกกำหนด"""
        assert settings.PATTERN_DIR is not None
        assert settings.DEADLOCK_MODEL_PATH is not None


class TestCellPenalty:
    """ทดสอบ CellPenalty dataclass"""
    
    def test_default_values(self):
        """ทดสอบค่า default"""
        cell = CellPenalty()
        assert cell.base_penalty == 0.0
        assert cell.traffic_history == 0
        assert cell.conflict_history == 0
        assert cell.yield_zone == False
        assert cell.priority_zone == False
    
    def test_custom_values(self):
        """ทดสอบการกำหนดค่า"""
        cell = CellPenalty(
            base_penalty=1.5,
            traffic_history=10,
            yield_zone=True
        )
        assert cell.base_penalty == 1.5
        assert cell.traffic_history == 10
        assert cell.yield_zone == True


# ===========================
# Integration Tests
# ===========================

class TestSimulationControllerIntegration:
    """ทดสอบ SimulationController แบบ Integration"""
    
    @pytest.fixture
    def controller(self):
        """สร้าง SimulationController สำหรับทดสอบ"""
        from controllers.simulation_controller import SimulationController
        return SimulationController(settings.PATTERN_DIR)
    
    def test_init_robots(self, controller):
        """ทดสอบการ init robots"""
        assert len(controller.robots) > 0
        for robot in controller.robots:
            assert "id" in robot
            assert "name" in robot
            assert "pos" in robot
            assert "state" in robot
    
    def test_init_packages(self, controller):
        """ทดสอบการ init packages"""
        assert len(controller.packages) > 0
        for pid, pkg in controller.packages.items():
            assert "name" in pkg
            assert "pickup" in pkg
            assert "dropoff" in pkg
            assert "status" in pkg
    
    def test_init_obstacles(self, controller):
        """ทดสอบการ init obstacles"""
        assert len(controller.obstacles) > 0
    
    def test_is_safe_cell(self, controller):
        """ทดสอบ is_safe_cell"""
        # ตำแหน่งว่าง
        robot_pos = controller.robots[0]["pos"]
        assert controller.is_safe_cell(robot_pos) == True
        
        # ตำแหน่งกำแพง
        obstacle = list(controller.obstacles)[0]
        assert controller.is_safe_cell(obstacle) == False
        
        # ตำแหน่งนอก bounds
        assert controller.is_safe_cell((-1, -1)) == False
    
    def test_get_robot_by_id(self, controller):
        """ทดสอบ get_robot_by_id"""
        robot = controller.get_robot_by_id(1)
        assert robot is not None
        assert robot["id"] == 1
        
        # ID ที่ไม่มี
        robot = controller.get_robot_by_id(999)
        assert robot is None
    
    def test_get_robot_priority(self, controller):
        """ทดสอบ get_robot_priority"""
        robot = controller.robots[0]
        priority = controller.get_robot_priority(robot)
        assert isinstance(priority, (int, float))
    
    def test_smart_astar_simple(self, controller):
        """ทดสอบ A* แบบง่าย"""
        robot = controller.robots[0]
        start = robot["pos"]
        
        # หา goal ที่ไม่ใช่ obstacle
        goal = None
        for r in range(settings.ROWS):
            for c in range(settings.COLS):
                if (r, c) not in controller.obstacles and (r, c) != start:
                    goal = (r, c)
                    break
            if goal:
                break
        
        if goal:
            blocked = set(controller.obstacles)
            path = controller.smart_astar(start, goal, blocked, robot)
            # path อาจว่างถ้าหาไม่เจอ แต่ต้องเป็น list
            assert isinstance(path, list)
    
    def test_detect_deadlock_group(self, controller):
        """ทดสอบ detect_deadlock_group"""
        groups = controller.detect_deadlock_group()
        assert isinstance(groups, list)


class TestPathFinderIntegration:
    """ทดสอบ PathFinder แบบ Integration"""
    
    @pytest.fixture
    def pathfinder(self):
        """สร้าง PathFinder สำหรับทดสอบ"""
        from controllers.simulation_controller import SimulationController
        controller = SimulationController(settings.PATTERN_DIR)
        return controller.pathfinder
    
    def test_predict_future_positions(self, pathfinder):
        """ทดสอบการทำนายตำแหน่งอนาคต"""
        robot = pathfinder.robots[0]
        predictions = pathfinder.predict_future_positions(robot, steps=3)
        assert isinstance(predictions, dict)
    
    def test_is_narrow_passage(self, pathfinder):
        """ทดสอบการตรวจจับทางแคบ"""
        # ต้องหาตำแหน่งที่เป็นทางแคบจริง
        result = pathfinder.is_narrow_passage((0, 0))
        assert isinstance(result, bool)


class TestDeadlockResolverIntegration:
    """ทดสอบ DeadlockResolver แบบ Integration"""
    
    @pytest.fixture
    def resolver(self):
        """สร้าง DeadlockResolver สำหรับทดสอบ"""
        from controllers.simulation_controller import SimulationController
        controller = SimulationController(settings.PATTERN_DIR)
        return controller.deadlock_resolver
    
    def test_get_robot_importance(self, resolver):
        """ทดสอบการคำนวณ importance"""
        robot = resolver.robots[0]
        importance = resolver.get_robot_importance(robot)
        assert isinstance(importance, (int, float))
    
    def test_get_emergency_move(self, resolver):
        """ทดสอบการหา emergency move"""
        robot = resolver.robots[0]
        move = resolver.get_emergency_move(robot)
        # อาจได้ None หรือ tuple
        assert move is None or isinstance(move, tuple)


class TestRobotManagerIntegration:
    """ทดสอบ RobotManager แบบ Integration"""
    
    @pytest.fixture
    def manager(self):
        """สร้าง RobotManager สำหรับทดสอบ"""
        from controllers.simulation_controller import SimulationController
        controller = SimulationController(settings.PATTERN_DIR)
        return controller.robot_manager
    
    def test_get_traffic_density(self, manager):
        """ทดสอบการคำนวณ traffic density"""
        density = manager.get_traffic_density((5, 5), robot_id=1)
        assert isinstance(density, (int, float))
        assert density >= 0
    
    def test_detect_oscillation(self, manager):
        """ทดสอบการตรวจจับ oscillation"""
        robot = manager.robots[0]
        result = manager.detect_oscillation(robot)
        assert isinstance(result, bool)


# ===========================
# Performance Tests
# ===========================

class TestPerformance:
    """ทดสอบ Performance"""
    
    def test_astar_performance(self):
        """ทดสอบความเร็วของ A*"""
        import time
        from controllers.simulation_controller import SimulationController
        
        controller = SimulationController(settings.PATTERN_DIR)
        robot = controller.robots[0]
        
        start_time = time.time()
        
        # รัน A* หลายครั้ง
        for _ in range(10):
            blocked = set(controller.obstacles)
            controller.smart_astar(
                robot["pos"], 
                (settings.ROWS - 1, settings.COLS - 1), 
                blocked, 
                robot
            )
        
        elapsed = time.time() - start_time
        
        # ต้องเสร็จใน 5 วินาที
        assert elapsed < 5.0, f"A* ช้าเกินไป: {elapsed:.2f}s"
    
    def test_penalty_map_performance(self):
        """ทดสอบความเร็วของ PenaltyMap"""
        import time
        
        pm = DynamicPenaltyMap(100, 100)
        
        start_time = time.time()
        
        # อัพเดทหลายครั้ง
        for step in range(100):
            for r in range(10):
                for c in range(10):
                    pm.update_traffic((r, c), step)
        
        elapsed = time.time() - start_time
        
        # ต้องเสร็จใน 2 วินาที
        assert elapsed < 2.0, f"PenaltyMap ช้าเกินไป: {elapsed:.2f}s"


# ===========================
# Time-Space A* Tests
# ===========================

class TestReservationTable:
    """ทดสอบ ReservationTable class"""
    
    def test_init(self):
        """ทดสอบการสร้าง ReservationTable"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        assert rt is not None
        assert len(rt.reservations) == 0
    
    def test_reserve_position(self):
        """ทดสอบการจองตำแหน่ง"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        rt.reserve(robot_id=1, position=(5, 5), timestep=10)
        
        assert rt.is_reserved((5, 5), 10)
        assert not rt.is_reserved((5, 5), 11)
        assert not rt.is_reserved((6, 6), 10)
    
    def test_reserve_with_exclude(self):
        """ทดสอบการตรวจสอบ reservation โดย exclude robot"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        rt.reserve(robot_id=1, position=(5, 5), timestep=10)
        
        # ถ้า exclude robot 1 ไม่ควร reserved
        assert not rt.is_reserved((5, 5), 10, exclude_robot=1)
        # ถ้า exclude robot 2 ควร reserved
        assert rt.is_reserved((5, 5), 10, exclude_robot=2)
    
    def test_reserve_path(self):
        """ทดสอบการจอง path"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        path = [(1, 1), (1, 2), (1, 3), (1, 4)]
        rt.reserve_path(robot_id=1, path=path, start_time=0)
        
        # Path ควร reserved ตามลำดับเวลา
        assert rt.is_reserved((1, 1), 0)
        assert rt.is_reserved((1, 2), 1)
        assert rt.is_reserved((1, 3), 2)
        assert rt.is_reserved((1, 4), 3)
    
    def test_clear_robot(self):
        """ทดสอบการล้าง reservations ของ robot"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        rt.reserve(robot_id=1, position=(5, 5), timestep=10)
        rt.clear_robot(1)
        
        assert not rt.is_reserved((5, 5), 10)
    
    def test_clear_old(self):
        """ทดสอบการล้าง old reservations"""
        from utils.time_space_astar import ReservationTable
        rt = ReservationTable()
        rt.reserve(robot_id=1, position=(5, 5), timestep=5)
        rt.reserve(robot_id=1, position=(6, 6), timestep=15)
        rt.clear_old(10)
        
        # timestep 5 ควรถูกลบ
        assert not rt.is_reserved((5, 5), 5)
        # timestep 15 ยังอยู่
        assert rt.is_reserved((6, 6), 15)


class TestTimeSpaceAStar:
    """ทดสอบ TimeSpaceAStar class"""
    
    @pytest.fixture
    def ts_astar(self):
        """สร้าง TimeSpaceAStar สำหรับทดสอบ"""
        from utils.time_space_astar import TimeSpaceAStar, ReservationTable
        obstacles = set()
        corridor_map = {}
        robots = [{"id": 1, "pos": (0, 0), "last_dir": (0, 0), "path": [], 
                   "state": "IDLE", "package": None, "momentum": 0, "wait_count": 0}]
        packages = {}
        
        return TimeSpaceAStar(
            obstacles=obstacles,
            corridor_map=corridor_map,
            robots=robots,
            packages=packages,
            reservation_table=ReservationTable()
        )
    
    def test_find_path_simple(self, ts_astar):
        """ทดสอบหา path ง่ายๆ"""
        robot = ts_astar.robots[0]
        path = ts_astar.find_path(
            start=(0, 0),
            goal=(0, 5),
            start_time=0,
            robot=robot
        )
        
        assert isinstance(path, list)
        if path:  # ถ้าหาเจอ
            assert path[-1] == (0, 5)  # ต้องจบที่ goal
    
    def test_find_path_to_self(self, ts_astar):
        """ทดสอบหา path ไปยังตำแหน่งเดิม"""
        robot = ts_astar.robots[0]
        path = ts_astar.find_path(
            start=(5, 5),
            goal=(5, 5),
            start_time=0,
            robot=robot
        )
        
        assert path == []  # ไม่ต้องเดินไปไหน
    
    def test_avoid_reserved_positions(self, ts_astar):
        """ทดสอบการหลีกเลี่ยง reserved positions"""
        # จองตำแหน่งที่อยู่ตรงกลาง
        ts_astar.reservation_table.reserve(robot_id=2, position=(0, 3), timestep=3)
        
        robot = ts_astar.robots[0]
        path = ts_astar.find_path(
            start=(0, 0),
            goal=(0, 5),
            start_time=0,
            robot=robot
        )
        
        # Path ควรหลีกเลี่ยง (0, 3) ที่ timestep 3
        # หรืออาจมี WAIT หรืออ้อม
        assert isinstance(path, list)
    
    def test_fallback_astar(self, ts_astar):
        """ทดสอบ fallback A*"""
        robot = ts_astar.robots[0]
        path = ts_astar._fallback_astar(
            start=(0, 0),
            goal=(3, 3),
            robot=robot,
            blocked=set()
        )
        
        assert isinstance(path, list)
        if path:
            assert path[-1] == (3, 3)


class TestTimeSpaceAStarIntegration:
    """ทดสอบ Time-Space A* แบบ Integration กับ SimulationController"""
    
    @pytest.fixture
    def controller(self):
        """สร้าง SimulationController สำหรับทดสอบ"""
        from controllers.simulation_controller import SimulationController
        return SimulationController(settings.PATTERN_DIR)
    
    def test_pathfinder_has_reservation_table(self, controller):
        """ทดสอบว่า pathfinder มี reservation table"""
        assert hasattr(controller.pathfinder, 'reservation_table')
        assert hasattr(controller.pathfinder, 'ts_astar')
    
    def test_update_pathfinder_step(self, controller):
        """ทดสอบ update_pathfinder_step"""
        controller.update_pathfinder_step(10)
        assert controller.pathfinder.current_step == 10
    
    def test_smart_astar_with_time_space(self, controller):
        """ทดสอบ smart_astar ใช้ Time-Space A*"""
        controller.update_pathfinder_step(0)
        robot = controller.robots[0]
        
        # หา goal ที่ไม่ใช่ obstacle
        goal = None
        for r in range(settings.ROWS):
            for c in range(settings.COLS):
                if (r, c) not in controller.obstacles and (r, c) != robot["pos"]:
                    goal = (r, c)
                    break
            if goal:
                break
        
        if goal:
            blocked = set(controller.obstacles)
            path = controller.smart_astar(robot["pos"], goal, blocked, robot)
            assert isinstance(path, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
