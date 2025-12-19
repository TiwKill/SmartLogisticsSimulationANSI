from core.settings import settings

class GridUtils:
    @staticmethod
    def parse_pos(pos_input):
        """
        แปลง input ให้เป็น tuple (row, col)
        รับค่าเฉพาะ list หรือ tuple ของตัวเลขเท่านั้น เช่น [5, 10]
        """
        if isinstance(pos_input, (list, tuple)):
            # แปลงเป็น int เพื่อความชัวร์
            return tuple(map(int, pos_input))
        
        # ยกเลิกการรองรับ String แบบ "A1" เพื่อความชัดเจนของข้อมูล
        raise ValueError(f"Invalid position format: {pos_input}. Expected [row, col].")

    @staticmethod
    def pos_to_str(pos):
        """แปลง (row, col) -> '[row, col]'"""
        return f"[{pos[0]}, {pos[1]}]"

    @staticmethod
    def in_bounds(r, c):
        return 0 <= r < settings.ROWS and 0 <= c < settings.COLS

    @staticmethod
    def manhattan(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    @staticmethod
    def create_wall(wall_def):
        """
        สร้าง set ของตำแหน่ง Wall
        รับค่า format: [r1, c1, r2, c2] เท่านั้น
        """
        if len(wall_def) == 4:
            r1, c1, r2, c2 = map(int, wall_def)
            # หา min/max เผื่อ user ใส่สลับตำแหน่ง
            r_start, r_end = min(r1, r2), max(r1, r2)
            c_start, c_end = min(c1, c2), max(c1, c2)
            
            wall_set = set()
            for r in range(r_start, r_end + 1):
                for c in range(c_start, c_end + 1):
                    wall_set.add((r, c))
            return wall_set
            
        raise ValueError(f"Invalid wall format: {wall_def}. Expected [r1, c1, r2, c2]")

    @staticmethod
    def get_direction(from_pos, to_pos):
        return (to_pos[0] - from_pos[0], to_pos[1] - from_pos[1])

    @staticmethod
    def is_turn(old_dir, new_dir):
        if old_dir == (0, 0):
            return False
        return old_dir != new_dir