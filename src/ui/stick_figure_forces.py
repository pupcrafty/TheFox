#!/usr/bin/env python3
"""
Force sequence system for stick figure
Handles preloaded force sequences from JSON and force fading
"""

import json
import math
from typing import Dict, List, Tuple, Optional


# Valid sensor targets - only these can receive forces
VALID_TARGETS = ["left_upper_arm", "right_upper_arm", "left_thigh", "right_thigh"]


class ForceSequence:
    """
    Represents a sequence of forces to apply over time
    Loaded from JSON file format
    """
    
    def __init__(self, name: str, force_definitions: List[Dict]):
        """
        Initialize force sequence
        
        Args:
            name: Name of the sequence
            force_definitions: List of force dictionaries from JSON
        """
        self.name = name
        self.force_definitions = force_definitions
        
        # Validate all targets are valid
        for force_def in force_definitions:
            target = force_def.get("target")
            if target not in VALID_TARGETS:
                raise ValueError(f"Invalid target '{target}' in sequence '{name}'. Must be one of {VALID_TARGETS}")
    
    def get_forces_at_time(self, sequence_time: float) -> List[Dict]:
        """
        Get forces that should be active at the given sequence time
        
        Args:
            sequence_time: Time since sequence started (seconds)
            
        Returns:
            List of force dictionaries that should be active
        """
        active_forces = []
        for force_def in self.force_definitions:
            force_time = force_def.get("time", 0.0)
            duration = force_def.get("duration", 0.0)
            
            # Check if this force should be active
            if force_time <= sequence_time < force_time + duration:
                active_forces.append(force_def)
        
        return active_forces


class ForceManager:
    """
    Manages force sequences and active forces
    Handles force fading and application to stick figure bodies
    """
    
    def __init__(self, stick_figure, config: Optional[Dict] = None):
        """
        Initialize force manager
        
        Args:
            stick_figure: StickFigure instance (to get body references)
            config: Optional configuration dictionary
        """
        self.stick_figure = stick_figure
        self.config = config or {}
        
        # Force sequences loaded from JSON
        self.sequences: Dict[str, ForceSequence] = {}
        
        # Currently playing sequence
        self.current_sequence: Optional[ForceSequence] = None
        self.sequence_start_time: float = 0.0
        self.sequence_playing: bool = False
        
        # Active forces: list of active force dicts
        # Format: {
        #     "target": str,  # Body part name
        #     "body": pymunk.Body,  # Body reference
        #     "force": (fx, fy),  # Force vector
        #     "torque": float,  # Torque
        #     "start_time": float,  # When force started (absolute time)
        #     "duration": float,  # How long force lasts
        #     "fade_type": str,  # "linear", "exponential", or "instant"
        #     "initial_force": (fx, fy),  # Initial force for fading
        #     "initial_torque": float  # Initial torque for fading
        # }
        self.active_forces: List[Dict] = []
        
        # Continuous forces (from peripheral devices, to be added later)
        self.continuous_forces: Dict[str, Tuple[float, float, float]] = {}
        
        # Fade rates from config
        self.fade_rate_exponential = self.config.get("forces", {}).get("fade_rate_exponential", 0.9)
        self.fade_rate_linear = self.config.get("forces", {}).get("fade_rate_linear", 1.0)
    
    def load_sequences_from_file(self, filepath: str):
        """
        Load force sequences from JSON file
        
        Args:
            filepath: Path to JSON file
        """
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            sequences_data = data.get("sequences", [])
            for seq_data in sequences_data:
                name = seq_data.get("name")
                forces = seq_data.get("forces", [])
                
                if name:
                    sequence = ForceSequence(name, forces)
                    self.sequences[name] = sequence
                    print(f"Loaded force sequence: {name} ({len(forces)} forces)")
        except Exception as e:
            print(f"Error loading force sequences from {filepath}: {e}")
    
    def play_sequence(self, sequence_name: str, current_time: float = 0.0):
        """
        Start playing a force sequence
        
        Args:
            sequence_name: Name of sequence to play
            current_time: Current time (for sequencing)
        """
        if sequence_name not in self.sequences:
            print(f"Warning: Force sequence '{sequence_name}' not found")
            return
        
        self.current_sequence = self.sequences[sequence_name]
        self.sequence_start_time = current_time
        self.sequence_playing = True
        print(f"Playing force sequence: {sequence_name}")
    
    def stop_sequence(self):
        """Stop currently playing sequence"""
        self.sequence_playing = False
        self.current_sequence = None
    
    def apply_force(self, target: str, force: Tuple[float, float], torque: float = 0.0, 
                   duration: float = 0.0, fade_type: str = "instant", current_time: float = 0.0):
        """
        Apply a force to a sensor target
        
        Args:
            target: Sensor target name (must be in VALID_TARGETS)
            force: Force vector (fx, fy)
            torque: Torque value (default: 0.0)
            duration: How long force lasts (0.0 = instant/one frame)
            fade_type: "linear", "exponential", or "instant"
            current_time: Current time for tracking
        """
        if target not in VALID_TARGETS:
            raise ValueError(f"Invalid target: {target}. Must be one of {VALID_TARGETS}")
        
        # Get body reference
        body = self._get_target_body(target)
        if body is None:
            print(f"Warning: Body for target '{target}' not found")
            return
        
        # Create active force entry
        active_force = {
            "target": target,
            "body": body,
            "force": force,
            "torque": torque,
            "start_time": current_time,
            "duration": duration,
            "fade_type": fade_type,
            "initial_force": force,
            "initial_torque": torque
        }
        
        self.active_forces.append(active_force)
    
    def set_continuous_force(self, target: str, force: Tuple[float, float], torque: float = 0.0):
        """
        Set a continuous force (from peripheral device)
        
        Args:
            target: Sensor target name (must be in VALID_TARGETS)
            force: Force vector (fx, fy)
            torque: Torque value (default: 0.0)
        """
        if target not in VALID_TARGETS:
            raise ValueError(f"Invalid target: {target}. Must be one of {VALID_TARGETS}")
        
        self.continuous_forces[target] = (force[0], force[1], torque)
    
    def clear_continuous_force(self, target: str):
        """
        Clear continuous force for a target
        
        Args:
            target: Sensor target name
        """
        if target in self.continuous_forces:
            del self.continuous_forces[target]
    
    def clear_all_forces(self, target: Optional[str] = None):
        """
        Clear all forces, or forces for a specific target
        
        Args:
            target: Optional target name. If None, clears all forces
        """
        if target is None:
            self.active_forces.clear()
            self.continuous_forces.clear()
        else:
            self.active_forces = [f for f in self.active_forces if f["target"] != target]
            if target in self.continuous_forces:
                del self.continuous_forces[target]
    
    def _get_target_body(self, target: str):
        """
        Get pymunk.Body reference for a target
        
        Args:
            target: Target name
            
        Returns:
            pymunk.Body or None
        """
        # Map target names to body names in stick_figure.bodies
        target_to_body_map = {
            "left_upper_arm": "left_upper_arm",
            "right_upper_arm": "right_upper_arm",
            "left_thigh": "left_thigh",
            "right_thigh": "right_thigh"
        }
        
        body_name = target_to_body_map.get(target)
        if body_name:
            return self.stick_figure.bodies.get(body_name)
        return None
    
    def _calculate_fade_factor(self, elapsed: float, duration: float, fade_type: str) -> float:
        """
        Calculate fade factor (0.0 to 1.0) based on elapsed time and fade type
        
        Args:
            elapsed: Time elapsed since force started
            duration: Total duration of force
            fade_type: "linear", "exponential", or "instant"
            
        Returns:
            Fade factor (1.0 = full strength, 0.0 = faded out)
        """
        if duration <= 0.0:
            return 1.0 if fade_type == "instant" else 0.0
        
        if elapsed >= duration:
            return 0.0
        
        progress = elapsed / duration
        
        if fade_type == "instant":
            return 1.0 if progress < 1.0 else 0.0
        elif fade_type == "linear":
            return 1.0 - progress
        elif fade_type == "exponential":
            # Exponential decay: factor = e^(-k*t)
            # k is chosen so that at t=duration, factor is approximately 0
            k = -math.log(0.01) / duration  # 0.01 = 1% remaining at end
            return math.exp(-k * elapsed)
        else:
            # Default to linear if unknown type
            return 1.0 - progress
    
    def update(self, dt: float, current_time: float):
        """
        Update force manager - handle sequence playback and force fading
        
        Args:
            dt: Delta time (seconds)
            current_time: Current absolute time (seconds)
        """
        # Update active sequence
        if self.sequence_playing and self.current_sequence:
            sequence_time = current_time - self.sequence_start_time
            
            # Get forces that should be active at this time
            active_force_defs = self.current_sequence.get_forces_at_time(sequence_time)
            
            # Check if any new forces need to be started
            for force_def in active_force_defs:
                target = force_def.get("target")
                force_time = force_def.get("time", 0.0)
                duration = force_def.get("duration", 0.0)
                
                # Check if this force is already active
                already_active = any(
                    f["target"] == target and 
                    abs(f["start_time"] - (self.sequence_start_time + force_time)) < 0.01
                    for f in self.active_forces
                )
                
                if not already_active:
                    # Start new force
                    force_vec = force_def.get("force", [0, 0])
                    torque = force_def.get("torque", 0.0)
                    fade_type = force_def.get("fade_type", "linear")
                    
                    self.apply_force(
                        target=target,
                        force=(force_vec[0], force_vec[1]),
                        torque=torque,
                        duration=duration,
                        fade_type=fade_type,
                        current_time=current_time
                    )
            
            # Check if sequence is done (no more forces will be activated)
            max_time = max((f.get("time", 0.0) + f.get("duration", 0.0) 
                           for f in self.current_sequence.force_definitions), default=0.0)
            if sequence_time >= max_time + 0.1:  # Small buffer
                self.stop_sequence()
        
        # Update active forces - apply fading and remove expired
        expired_indices = []
        for i, active_force in enumerate(self.active_forces):
            elapsed = current_time - active_force["start_time"]
            duration = active_force["duration"]
            fade_type = active_force["fade_type"]
            
            # Calculate fade factor
            fade_factor = self._calculate_fade_factor(elapsed, duration, fade_type)
            
            if fade_factor <= 0.01 or elapsed >= duration:
                # Force has expired
                expired_indices.append(i)
            else:
                # Apply faded force
                initial_force = active_force["initial_force"]
                initial_torque = active_force["initial_torque"]
                
                faded_force = (initial_force[0] * fade_factor, initial_force[1] * fade_factor)
                faded_torque = initial_torque * fade_factor
                
                body = active_force["body"]
                body.apply_force_at_local_point(faded_force, (0, 0))
                if abs(faded_torque) > 0.001:
                    body.torque += faded_torque
        
        # Remove expired forces (in reverse order to maintain indices)
        for i in reversed(expired_indices):
            self.active_forces.pop(i)
        
        # Apply continuous forces (from peripheral devices)
        for target, (fx, fy, torque) in self.continuous_forces.items():
            body = self._get_target_body(target)
            if body:
                body.apply_force_at_local_point((fx, fy), (0, 0))
                if abs(torque) > 0.001:
                    body.torque += torque
