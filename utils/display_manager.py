"""
Display Manager Module
จัดการการแสดงผลและ Activity Log สำหรับ Smart Logistics Simulation
"""

import time
from collections import deque
from core.settings import settings
from utils.grid_utils import GridUtils


class DisplayManager:
    """จัดการการแสดงผลและ Activity Log"""
    
    def __init__(self, max_activities=8):
        self.activities = deque(maxlen=max_activities)
        self.start_time = time.time()
        self.total_moves = 0
        self.total_pickups = 0
        self.total_dropoffs = 0
        self.deadlock_count = 0
        self.yield_count = 0
    
    def add_activity(self, message):
        timestamp = time.strftime('%H:%M:%S')
        self.activities.append(f"[{timestamp}] {message}")
    
    def get_activities(self):
        return list(self.activities)
    
    def record_move(self):
        self.total_moves += 1
    
    def record_pickup(self):
        self.total_pickups += 1
    
    def record_dropoff(self):
        self.total_dropoffs += 1
    
    def record_deadlock(self):
        self.deadlock_count += 1
    
    def record_yield(self):
        self.yield_count += 1
    
    def get_elapsed_time(self):
        return time.time() - self.start_time


class ANSIColors:
    """ANSI Color Codes สำหรับ Terminal Display"""
    # Foreground (Text)
    HEADER = '\033[95m'
    WHITE = '\033[97m'
    BLACK = '\033[30m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    MAGENTA = '\033[95m'
    BLUE = '\033[94m'
    DIM = '\033[90m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'
    
    # Backgrounds (For Robot States)
    BG_WALL = '\033[100m'   # Dark Gray BG
    BG_IDLE = '\033[44m'    # Blue BG (ว่าง)
    BG_PICKUP = '\033[46m'  # Cyan BG (กำลังไปรับของ)
    BG_DROPOFF = '\033[42m' # Green BG (มีของ/ไปส่ง)
    BG_EVAC = '\033[41m'    # Red BG (หลบ/ฉุกเฉิน)
    BG_HOME = '\033[45m'    # Magenta BG (กลับบ้าน)


class SimulationRenderer:
    """จัดการการแสดงผล Grid และ Statistics"""
    
    def __init__(self, display_manager: DisplayManager):
        self.display = display_manager
        self.C = ANSIColors
    
    def render(self, step, robots, packages, obstacles, corridor_map):
        """แสดงผล Grid และ Statistics"""
        C = self.C
        
        # --- 1. Clear Screen & Header ---
        print("\033[H\033[J", end="") 
        print(f"{C.HEADER}{C.BOLD}╔{'═'*100}╗{C.ENDC}")
        print(f"{C.HEADER}{C.BOLD}║{'🤖 SMART LOGISTICS SIMULATION 🤖':^100}║{C.ENDC}")
        print(f"{C.HEADER}{C.BOLD}╚{'═'*100}╝{C.ENDC}")
        
        # --- Statistics Bar ---
        elapsed = self.display.get_elapsed_time()
        elapsed_str = f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"
        delivered_count = sum(1 for p in packages.values() if p["status"] == "DELIVERED")
        total_pkgs = len(packages)
        
        print(f" {C.BOLD}Step:{C.ENDC} {C.CYAN}{step:<5}{C.ENDC} | "
              f"{C.BOLD}Time:{C.ENDC} {elapsed_str} | "
              f"{C.BOLD}Moves:{C.ENDC} {self.display.total_moves} | "
              f"{C.BOLD}Pickups:{C.ENDC} {C.GREEN}{self.display.total_pickups}{C.ENDC} | "
              f"{C.BOLD}Dropoffs:{C.ENDC} {C.YELLOW}{self.display.total_dropoffs}{C.ENDC} | "
              f"{C.BOLD}Deadlocks:{C.ENDC} {C.RED}{self.display.deadlock_count}{C.ENDC}")
        print("─" * 100)

        # --- 2. Prepare Grid Data ---
        grid_display = [[f"{C.DIM} · {C.ENDC}" for _ in range(settings.COLS)] for _ in range(settings.ROWS)]

        # Walls (Racks)
        for r, c in obstacles:
            if GridUtils.in_bounds(r, c):
                grid_display[r][c] = f"{C.BG_WALL}   {C.ENDC}"

        # Packages (Pickup/Dropoff)
        for pkg in packages.values():
            if pkg["status"] == "WAITING":
                r, c = pkg["pickup"]
                if GridUtils.in_bounds(r, c):
                    grid_display[r][c] = f"{C.GREEN}{C.BOLD} P {C.ENDC}"
            
            if pkg["status"] in ["WAITING", "PICKED"]:
                r, c = pkg["dropoff"]
                if GridUtils.in_bounds(r, c):
                    grid_display[r][c] = f"{C.YELLOW}{C.BOLD} D {C.ENDC}"

        # Robots
        for rb in robots:
            r, c = rb["pos"]
            if GridUtils.in_bounds(r, c):
                short_name = rb["name"].replace("R", "")
                display_text = f"{short_name:^3}"
                
                # เลือกสีตามสถานะ (State Machine Colors)
                state = rb['state']
                if state == "TO_PICKUP":
                    style = C.BG_PICKUP
                elif state == "TO_DROPOFF":
                    style = C.BG_DROPOFF
                elif state == "EVACUATING":
                    style = C.BG_EVAC
                elif state == "HOME":
                    style = C.BG_HOME
                else:  # IDLE
                    style = C.BG_IDLE
                
                grid_display[r][c] = f"{style}{C.BOLD}{C.WHITE}{display_text}{C.ENDC}"

        # --- 3. Print Grid ---
        indent = "   "
        col_header = indent + "".join(f"{i:02} " for i in range(settings.COLS))
        print(f"{C.DIM}{col_header}{C.ENDC}")

        for i, row in enumerate(grid_display):
            row_label = f"{C.DIM}{i:02} {C.ENDC}" 
            print(f"{row_label}" + "".join(row))

        print("─" * 100)
        
        # --- 4. LEGEND ---
        print(f"{C.BOLD}📋 LEGEND:{C.ENDC} ", end="")
        print(f"{C.BG_IDLE}{C.WHITE} IDLE {C.ENDC} ", end="")
        print(f"{C.BG_PICKUP}{C.WHITE} TO_PICKUP {C.ENDC} ", end="")
        print(f"{C.BG_DROPOFF}{C.WHITE} DELIVERING {C.ENDC} ", end="")
        print(f"{C.BG_HOME}{C.WHITE} HOME {C.ENDC} ", end="")
        print(f"{C.BG_EVAC}{C.WHITE} EVAC/YIELD {C.ENDC} ", end="")
        print(f"{C.GREEN}{C.BOLD}P{C.ENDC}=Pickup ", end="")
        print(f"{C.YELLOW}{C.BOLD}D{C.ENDC}=Dropoff")
        print()
        
        # --- 5. STATISTICS ---
        progress = int((delivered_count / total_pkgs) * 40) if total_pkgs > 0 else 0
        bar = "█" * progress + "░" * (40 - progress)
        pct = (delivered_count / total_pkgs * 100) if total_pkgs > 0 else 0
        
        print(f"{C.BOLD}📊 STATISTICS:{C.ENDC}")
        print(f"   Progress: {delivered_count}/{total_pkgs} ({pct:.1f}%) [{C.GREEN}{bar}{C.ENDC}]")
        
        # Count robot states
        state_counts = {"IDLE": 0, "TO_PICKUP": 0, "TO_DROPOFF": 0, "HOME": 0, "EVACUATING": 0}
        for rb in robots:
            state_counts[rb["state"]] = state_counts.get(rb["state"], 0) + 1
        
        print(f"   Robots: IDLE={C.BLUE}{state_counts['IDLE']}{C.ENDC} | "
              f"TO_PICKUP={C.CYAN}{state_counts['TO_PICKUP']}{C.ENDC} | "
              f"DELIVERING={C.GREEN}{state_counts['TO_DROPOFF']}{C.ENDC} | "
              f"HOME={C.MAGENTA}{state_counts['HOME']}{C.ENDC} | "
              f"EVAC={C.RED}{state_counts['EVACUATING']}{C.ENDC}")
        print()
        
        # --- 6. PACKAGE STATUS ---
        print(f"{C.BOLD}📦 PACKAGE STATUS:{C.ENDC}")
        waiting = [p for p in packages.values() if p["status"] == "WAITING"]
        picked = [p for p in packages.values() if p["status"] == "PICKED"]
        delivered = [p for p in packages.values() if p["status"] == "DELIVERED"]
        
        print(f"   {C.YELLOW}WAITING ({len(waiting)}):{C.ENDC} ", end="")
        print(", ".join([p["name"] for p in waiting[:8]]) if waiting else "-", end="")
        if len(waiting) > 8: print(f" +{len(waiting)-8} more", end="")
        print()
        
        print(f"   {C.CYAN}IN TRANSIT ({len(picked)}):{C.ENDC} ", end="")
        print(", ".join([p["name"] for p in picked[:8]]) if picked else "-", end="")
        if len(picked) > 8: print(f" +{len(picked)-8} more", end="")
        print()
        
        print(f"   {C.GREEN}DELIVERED ({len(delivered)}):{C.ENDC} ", end="")
        print(", ".join([p["name"] for p in delivered[-8:]]) if delivered else "-", end="")
        if len(delivered) > 8: print(f" (+{len(delivered)-8} earlier)", end="")
        print()
        print()
        
        # --- 7. ROBOT STATUS TABLE ---
        print(f"{C.BOLD}🤖 ROBOT STATUS:{C.ENDC}")
        print(f"   {C.DIM}{'NAME':<6} {'STATE':<12} {'PACKAGE':<8} {'POSITION':<10} {'WAIT':<6} {'MODE':<10} {'PATH LEN'}{C.ENDC}")
        print(f"   {'─'*70}")
        
        for rb in robots:
            pkg_str = packages[rb["package"]]['name'] if rb["package"] is not None else "-"
            pos_str = GridUtils.pos_to_str(rb['pos'])
            path_len = len(rb["path"]) if rb["path"] else 0
            
            # Color based on state
            if rb['state'] == "TO_PICKUP": 
                state_color = C.CYAN
            elif rb['state'] == "TO_DROPOFF": 
                state_color = C.GREEN
            elif rb['state'] == "EVACUATING": 
                state_color = C.RED
            elif rb['state'] == "HOME": 
                state_color = C.MAGENTA
            else: 
                state_color = C.BLUE

            wait_str = str(rb['wait_count'])
            if rb['wait_count'] > 2:
                wait_str = f"{C.RED}{rb['wait_count']}{C.ENDC}"

            mode_str = rb['decision_mode']
            if mode_str != "NORMAL": 
                mode_str = f"{C.YELLOW}{mode_str}{C.ENDC}"
            else: 
                mode_str = f"{C.DIM}{mode_str}{C.ENDC}"

            print(f"   {rb['name']:<6} {state_color}{rb['state']:<12}{C.ENDC} {pkg_str:<8} {pos_str:<10} {wait_str:<6} {mode_str:<10} {path_len}")
        print()
        
        # --- 8. RECENT ACTIVITY ---
        print(f"{C.BOLD}📝 RECENT ACTIVITY:{C.ENDC}")
        activities = self.display.get_activities()
        if activities:
            for act in activities[-6:]:
                print(f"   {C.DIM}{act}{C.ENDC}")
        else:
            print(f"   {C.DIM}No recent activity{C.ENDC}")
        print("─" * 100)

    def render_final_statistics(self, total_steps, robots, packages):
        """แสดงสถิติสรุปเมื่อจบ simulation"""
        C = self.C
        
        elapsed = self.display.get_elapsed_time()
        delivered = sum(1 for p in packages.values() if p["status"] == "DELIVERED")
        total_turns = sum(rb['total_turns'] for rb in robots)
        
        print()
        print(f"{C.GREEN}{C.BOLD}╔{'═'*60}╗{C.ENDC}")
        print(f"{C.GREEN}{C.BOLD}║{'🎉 SIMULATION COMPLETE 🎉':^60}║{C.ENDC}")
        print(f"{C.GREEN}{C.BOLD}╚{'═'*60}╝{C.ENDC}")
        print()
        print(f"{C.BOLD}📊 FINAL STATISTICS:{C.ENDC}")
        print(f"   ┌{'─'*40}┐")
        print(f"   │ {'Total Steps:':<25} {C.CYAN}{total_steps:>12}{C.ENDC} │")
        print(f"   │ {'Elapsed Time:':<25} {C.CYAN}{int(elapsed//60):02d}:{int(elapsed%60):02d}{' '*8}{C.ENDC} │")
        print(f"   │ {'Packages Delivered:':<25} {C.GREEN}{delivered:>12}{C.ENDC} │")
        print(f"   │ {'Total Moves:':<25} {C.YELLOW}{self.display.total_moves:>12}{C.ENDC} │")
        print(f"   │ {'Total Pickups:':<25} {C.GREEN}{self.display.total_pickups:>12}{C.ENDC} │")
        print(f"   │ {'Total Dropoffs:':<25} {C.GREEN}{self.display.total_dropoffs:>12}{C.ENDC} │")
        print(f"   │ {'Total Turns:':<25} {C.MAGENTA}{total_turns:>12}{C.ENDC} │")
        print(f"   │ {'Deadlocks Resolved:':<25} {C.RED}{self.display.deadlock_count:>12}{C.ENDC} │")
        print(f"   │ {'Yield Events:':<25} {C.YELLOW}{self.display.yield_count:>12}{C.ENDC} │")
        print(f"   └{'─'*40}┘")
        print()
        
        # Efficiency metrics
        if total_steps > 0:
            efficiency = (delivered / total_steps) * 100
            moves_per_pkg = self.display.total_moves / delivered if delivered > 0 else 0
            print(f"{C.BOLD}📈 EFFICIENCY METRICS:{C.ENDC}")
            print(f"   Delivery Rate: {C.GREEN}{efficiency:.2f}%{C.ENDC} per step")
            print(f"   Moves per Package: {C.CYAN}{moves_per_pkg:.1f}{C.ENDC}")
            print(f"   Avg Steps per Delivery: {C.CYAN}{total_steps/delivered:.1f}{C.ENDC}" if delivered > 0 else "")
        
        print()
        print(f"{C.DIM}{'─'*60}{C.ENDC}")
        print(f"{C.BOLD}All robots returned home. Simulation ended successfully!{C.ENDC}")
        print()
