#!/usr/bin/env python3
"""
CARLA Simulation MTTF and MTTR Calculator

This script processes JSONL files from CARLA simulations to calculate:
- Mean Time to Failure (MTTF)
- Mean Time to Repair (MTTR)

Failure conditions:
1. Collision detected
2. Vehicle deviates from waypoint path beyond threshold
3. Vehicle speed exceeds speed limit by more than 10 mph
"""

import json
import argparse
import sys
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from pathlib import Path


@dataclass
class FailureEvent:
    """Represents a failure event in the simulation"""
    failure_time: float
    failure_type: str
    is_fatal: bool  # True if collision, False otherwise
    frame: int
    repair_time: Optional[float] = None
    repair_frame: Optional[int] = None
    
    @property
    def ttf(self) -> float:
        """Time to Failure from simulation start"""
        return self.failure_time
    
    @property
    def ttr(self) -> Optional[float]:
        """Time to Repair (recovery time after failure)"""
        if self.repair_time is not None:
            return self.repair_time - self.failure_time
        return None


class CarlaSimulationAnalyzer:
    """Analyzes CARLA simulation data for MTTF and MTTR metrics"""
    
    def __init__(self, 
                 waypoint_distance_threshold: float = 5.0,
                 speed_limit_excess_mph: float = 10.0,
                 check_interval: int = 10,
                 recovery_speed_threshold_kph: float = 5.0,
                 recovery_distance_threshold: float = 3.0):
        """
        Initialize the analyzer with thresholds
        
        Args:
            waypoint_distance_threshold: Max distance (meters) from waypoint path
            speed_limit_excess_mph: Speeding threshold (mph over limit)
            check_interval: Check failures every N frames
            recovery_speed_threshold_kph: Speed variation threshold for recovery (kph)
            recovery_distance_threshold: Distance threshold for recovery (meters)
        """
        self.waypoint_threshold = waypoint_distance_threshold
        self.speed_excess_mph = speed_limit_excess_mph
        self.speed_excess_kph = speed_limit_excess_mph * 1.60934  # Convert to kph
        self.check_interval = check_interval
        self.recovery_speed_threshold = recovery_speed_threshold_kph
        self.recovery_distance_threshold = recovery_distance_threshold
        
        self.failures: List[FailureEvent] = []
        self.in_failure_state = False
        self.current_failure: Optional[FailureEvent] = None
        self.stable_ticks = 0
        self.required_stable_ticks = 3  # Require 3 consecutive stable ticks for recovery
        
    def load_simulation_data(self, jsonl_file: str) -> List[Dict]:
        """Load CARLA simulation data from JSONL file"""
        data = []
        try:
            with open(jsonl_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line:
                        data.append(json.loads(line))
            print(f"Loaded {len(data)} records from {jsonl_file}")
            return data
        except FileNotFoundError:
            print(f"Error: File {jsonl_file} not found")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error parsing JSON: {e}")
            sys.exit(1)
    
    def check_collision(self, record: Dict) -> bool:
        """Check if a collision occurred in this frame"""
        collisions = record.get('collisions_this_frame', [])
        return len(collisions) > 0
    
    def calculate_waypoint_distance(self, record: Dict) -> Optional[float]:
        """
        Calculate distance from vehicle to reference waypoint path
        Uses the lane_deviation_m field directly from CARLA data
        """
        # CARLA provides lane_deviation_m which is the distance from center of lane
        return record.get('lane_deviation_m', None)
    
    def check_speed_violation(self, record: Dict) -> bool:
        """Check if vehicle exceeded speed limit by threshold (10 mph)"""
        speed_kph = record.get('speed_kph', 0)
        speed_limit_kph = record.get('speed_limit_kph', 30.0)
        
        # Check if speed exceeds limit by more than threshold
        excess_kph = speed_kph - speed_limit_kph
        return excess_kph > self.speed_excess_kph
    
    def is_vehicle_stable(self, record: Dict, recent_records: List[Dict]) -> bool:
        """
        Check if vehicle has returned to stable state
        
        Stability criteria:
        - Lane deviation within acceptable range
        - Speed within acceptable range (not exceeding limit)
        - Speed relatively constant over recent frames
        """
        # Check lane deviation
        lane_deviation = self.calculate_waypoint_distance(record)
        if lane_deviation is None:
            waypoint_stable = True  # Can't check, assume stable
        else:
            waypoint_stable = lane_deviation <= self.recovery_distance_threshold
        
        # Check speed is not violating
        speed_stable = not self.check_speed_violation(record)
        
        # Check speed is relatively constant (if we have recent data)
        if len(recent_records) >= 2:
            current_speed = record.get('speed_kph', 0)
            prev_speed = recent_records[-1].get('speed_kph', 0)
            speed_change = abs(current_speed - prev_speed)
            speed_constant = speed_change < self.recovery_speed_threshold
        else:
            speed_constant = True
        
        return waypoint_stable and speed_stable and speed_constant
    
    def analyze_simulation(self, data: List[Dict]) -> None:
        """
        Analyze simulation data to detect failures and recoveries
        
        Args:
            data: List of simulation records
        """
        self.failures = []
        self.in_failure_state = False
        self.current_failure = None
        recent_records = []
        
        # Get initial timestamp
        if not data:
            print("Warning: No data to analyze")
            return
        
        initial_time = data[0]['timestamp']['elapsed_seconds']
        
        for i, record in enumerate(data):
            # Get timestamp and frame
            elapsed_time = record['timestamp']['elapsed_seconds']
            relative_time = elapsed_time - initial_time
            frame = record.get('frame', i)
            
            # Keep track of recent records for stability check
            recent_records.append(record)
            if len(recent_records) > 5:
                recent_records.pop(0)
            
            # Check failures at specified intervals
            if i % self.check_interval == 0 or i == 0:
                # Check for collision (fatal)
                if self.check_collision(record):
                    # If there was an active non-fatal failure, save it first
                    if self.in_failure_state and self.current_failure:
                        print(f"  [Frame {frame}] Collision interrupted ongoing {self.current_failure.failure_type}")
                        self.failures.append(self.current_failure)
                    
                    # Record the collision
                    failure = FailureEvent(
                        failure_time=relative_time,
                        failure_type='collision',
                        is_fatal=True,
                        frame=frame
                    )
                    self.failures.append(failure)
                    print(f"  [Frame {frame}] Collision at t={failure.failure_time:.2f}s")
                    # Reset failure state since collision is terminal
                    self.in_failure_state = False
                    self.current_failure = None
                    continue
                
                # If not in failure state, check for new failures
                if not self.in_failure_state:
                    # Check waypoint deviation
                    lane_deviation = self.calculate_waypoint_distance(record)
                    if lane_deviation is not None and lane_deviation > self.waypoint_threshold:
                        self.current_failure = FailureEvent(
                            failure_time=relative_time,
                            failure_type='lane_deviation',
                            is_fatal=False,
                            frame=frame
                        )
                        self.in_failure_state = True
                        self.stable_ticks = 0
                        print(f"  [Frame {frame}] Lane deviation failure at t={self.current_failure.failure_time:.2f}s (deviation={lane_deviation:.3f}m)")
                    
                    # Check speed violation
                    elif self.check_speed_violation(record):
                        speed_kph = record.get('speed_kph', 0)
                        speed_limit_kph = record.get('speed_limit_kph', 30.0)
                        self.current_failure = FailureEvent(
                            failure_time=relative_time,
                            failure_type='speed_violation',
                            is_fatal=False,
                            frame=frame
                        )
                        self.in_failure_state = True
                        self.stable_ticks = 0
                        print(f"  [Frame {frame}] Speed violation at t={self.current_failure.failure_time:.2f}s (speed={speed_kph:.1f} kph, limit={speed_limit_kph:.1f} kph)")
                
                # If in failure state, check for recovery
                else:
                    if self.is_vehicle_stable(record, recent_records):
                        self.stable_ticks += 1
                        if self.stable_ticks >= self.required_stable_ticks:
                            # Vehicle has recovered
                            self.current_failure.repair_time = relative_time
                            self.current_failure.repair_frame = frame
                            self.failures.append(self.current_failure)
                            print(f"  [Frame {frame}] Recovery at t={self.current_failure.repair_time:.2f}s (TTR={self.current_failure.ttr:.2f}s)")
                            self.in_failure_state = False
                            self.current_failure = None
                            self.stable_ticks = 0
                    else:
                        self.stable_ticks = 0  # Reset if not stable
        
        # If simulation ended while in failure state
        if self.in_failure_state and self.current_failure:
            print(f"  Simulation ended in failure state (no recovery detected)")
            self.failures.append(self.current_failure)
    
    def calculate_mttf(self) -> Tuple[float, int]:
        """
        Calculate Mean Time to Failure (MTTF)
        
        MTTF = (1/N) * Σ(TTF_i) where TTF_i = T_failure - T_initial
        
        Returns:
            (mttf, num_failures): MTTF value and number of failures
        """
        if not self.failures:
            return 0.0, 0
        
        ttf_values = [f.ttf for f in self.failures]
        mttf = sum(ttf_values) / len(ttf_values)
        return mttf, len(ttf_values)
    
    def calculate_mttr(self) -> Tuple[float, int]:
        """
        Calculate Mean Time to Repair (MTTR)
        
        MTTR = (1/N) * Σ(TTR_i) where TTR_i = T_repair - T_failure
        Only considers non-fatal failures that recovered.
        
        Returns:
            (mttr, num_repairs): MTTR value and number of repairs
        """
        ttr_values = [f.ttr for f in self.failures 
                      if not f.is_fatal and f.ttr is not None]
        
        if not ttr_values:
            return 0.0, 0
        
        mttr = sum(ttr_values) / len(ttr_values)
        return mttr, len(ttr_values)
    
    def print_summary(self) -> None:
        """Print summary of analysis"""
        print("\n" + "="*70)
        print("CARLA SIMULATION RELIABILITY ANALYSIS")
        print("="*70)
        
        print(f"\nTotal failures detected: {len(self.failures)}")
        
        # Breakdown by type
        collision_count = sum(1 for f in self.failures if f.is_fatal)
        lane_dev_count = sum(1 for f in self.failures if f.failure_type == 'lane_deviation')
        speed_count = sum(1 for f in self.failures if f.failure_type == 'speed_violation')
        
        print(f"  - Collisions (fatal): {collision_count}")
        print(f"  - Lane deviations: {lane_dev_count}")
        print(f"  - Speed violations: {speed_count}")
        
        # Show individual failure details
        if self.failures:
            print(f"\nFailure Events:")
            for i, f in enumerate(self.failures, 1):
                ttr_str = f"TTR={f.ttr:.2f}s" if f.ttr is not None else "No recovery"
                print(f"  {i}. Frame {f.frame}: {f.failure_type} at t={f.ttf:.2f}s, {ttr_str}")
        
        # MTTF
        print(f"\n{'─'*70}")
        mttf, num_failures = self.calculate_mttf()
        print(f"Mean Time to Failure (MTTF):")
        if num_failures > 0:
            print(f"  MTTF = {mttf:.3f} seconds")
            print(f"  Based on {num_failures} failure event(s)")
            print(f"  Formula: MTTF = (1/{num_failures}) × Σ(TTF_i)")
        else:
            print(f"  No failures detected")
        
        # MTTR
        print(f"\n{'─'*70}")
        mttr, num_repairs = self.calculate_mttr()
        print(f"Mean Time to Repair (MTTR):")
        if num_repairs > 0:
            print(f"  MTTR = {mttr:.3f} seconds")
            print(f"  Based on {num_repairs} repair(s)")
            print(f"  Formula: MTTR = (1/{num_repairs}) × Σ(TTR_i)")
            
            # Show repair details
            repairs = [f for f in self.failures if not f.is_fatal and f.ttr is not None]
            if repairs:
                print(f"\n  Repair breakdown:")
                for i, r in enumerate(repairs, 1):
                    print(f"    {i}. {r.failure_type}: TTR = {r.ttr:.3f}s")
        else:
            non_fatal = sum(1 for f in self.failures if not f.is_fatal)
            if non_fatal > 0:
                print(f"  N/A - {non_fatal} non-fatal failure(s) did not recover")
            else:
                print(f"  N/A - No non-fatal failures detected")
        
        print("\n" + "="*70)
    
    def export_results(self, output_file: str, simulation_name: str = None) -> None:
        """
        Export results to JSON file, appending to existing runs if file exists
        
        Args:
            output_file: Path to output JSON file
            simulation_name: Optional name for this simulation run
        """
        mttf, num_failures = self.calculate_mttf()
        mttr, num_repairs = self.calculate_mttr()
        
        # Create result for this run
        run_result = {
            "simulation_name": simulation_name,
            "mttf_seconds": mttf,
            "mttr_seconds": mttr,
            "total_failures": num_failures,
            "total_repairs": num_repairs,
            "collision_count": sum(1 for f in self.failures if f.is_fatal),
            "lane_deviation_count": sum(1 for f in self.failures if f.failure_type == 'lane_deviation'),
            "speed_violation_count": sum(1 for f in self.failures if f.failure_type == 'speed_violation'),
            "failures": [
                {
                    "frame": f.frame,
                    "type": f.failure_type,
                    "is_fatal": f.is_fatal,
                    "ttf_seconds": f.ttf,
                    "ttr_seconds": f.ttr,
                    "repair_frame": f.repair_frame
                }
                for f in self.failures
            ],
            "configuration": {
                "waypoint_threshold_m": self.waypoint_threshold,
                "speed_excess_mph": self.speed_excess_mph,
                "speed_excess_kph": self.speed_excess_kph,
                "check_interval_frames": self.check_interval
            }
        }
        
        # Load existing data if file exists
        all_runs = {"runs": []}
        if Path(output_file).exists():
            try:
                with open(output_file, 'r') as f:
                    all_runs = json.load(f)
                    # Ensure it has the expected structure
                    if "runs" not in all_runs:
                        all_runs = {"runs": []}
            except (json.JSONDecodeError, IOError):
                # If file is corrupted or empty, start fresh
                all_runs = {"runs": []}
        
        # Append this run
        all_runs["runs"].append(run_result)
        
        # Add summary statistics across all runs
        all_mttf = [r["mttf_seconds"] for r in all_runs["runs"] if r["mttf_seconds"] > 0]
        all_mttr = [r["mttr_seconds"] for r in all_runs["runs"] if r["mttr_seconds"] > 0]
        
        all_runs["summary"] = {
            "total_runs": len(all_runs["runs"]),
            "average_mttf_seconds": sum(all_mttf) / len(all_mttf) if all_mttf else 0.0,
            "average_mttr_seconds": sum(all_mttr) / len(all_mttr) if all_mttr else 0.0,
            "total_failures_all_runs": sum(r["total_failures"] for r in all_runs["runs"]),
            "total_repairs_all_runs": sum(r["total_repairs"] for r in all_runs["runs"])
        }
        
        # Write back to file
        with open(output_file, 'w') as f:
            json.dump(all_runs, f, indent=2)
        
        print(f"\nResults appended to: {output_file}")
        print(f"Total runs in file: {all_runs['summary']['total_runs']}")


def main():
    parser = argparse.ArgumentParser(
        description='Calculate MTTF and MTTR from CARLA simulation JSONL files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s simulation.jsonl
  %(prog)s simulation.jsonl --waypoint-threshold 3.0 --speed-excess 15.0
  %(prog)s simulation.jsonl --output results.json
        """
    )
    parser.add_argument('input_file', type=str, 
                       help='Path to CARLA simulation JSONL file')
    parser.add_argument('--waypoint-threshold', type=float, default=5.0,
                       help='Lane deviation threshold in meters (default: 5.0)')
    parser.add_argument('--speed-excess', type=float, default=10.0,
                       help='Speed limit excess threshold in mph (default: 10.0)')
    parser.add_argument('--check-interval', type=int, default=10,
                       help='Check failures every N frames (default: 10)')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Export results to JSON file (appends to existing runs)')
    parser.add_argument('--run-name', type=str, default=None,
                       help='Name for this simulation run (for identification in JSON output)')
    
    args = parser.parse_args()
    
    # Validate input file exists
    if not Path(args.input_file).exists():
        print(f"Error: Input file '{args.input_file}' not found")
        sys.exit(1)
    
    # Create analyzer
    analyzer = CarlaSimulationAnalyzer(
        waypoint_distance_threshold=args.waypoint_threshold,
        speed_limit_excess_mph=args.speed_excess,
        check_interval=args.check_interval
    )
    
    # Load and analyze data
    print(f"\nAnalyzing: {args.input_file}")
    print(f"Configuration:")
    print(f"  - Lane deviation threshold: {args.waypoint_threshold} m")
    print(f"  - Speed excess threshold: {args.speed_excess} mph ({args.speed_excess * 1.60934:.1f} kph)")
    print(f"  - Check interval: every {args.check_interval} frames")
    print(f"\nProcessing...")
    
    data = analyzer.load_simulation_data(args.input_file)
    analyzer.analyze_simulation(data)
    analyzer.print_summary()
    
    # Export if requested
    if args.output:
        run_name = args.run_name if args.run_name else Path(args.input_file).stem
        analyzer.export_results(args.output, simulation_name=run_name)


if __name__ == '__main__':
    main()