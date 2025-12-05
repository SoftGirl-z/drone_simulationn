# controller.py
"""
User input and controller interface
Converts keyboard input to control references
"""

import pygame
import math


class ManualController:
    """
    Keyboard-based manual control for quadcopter
    """
    
    def __init__(self):
        """Initialize controller with default values"""
        # Reference values
        self.z_ref = 2.5          # altitude setpoint (m)
        self.roll_ref = 0.0       # roll setpoint (rad)
        self.pitch_ref = 0.0      # pitch setpoint (rad)
        self.yaw_ref = 0.0        # yaw setpoint (rad)
        
        # Control parameters
        self.alt_step = 0.1       # altitude increment per step
        self.angle_step = 0.08    # angle increment per step (artırıldı: 0.03→0.08)
        
        # Limits (much more conservative than quadcopter)
        self.MAX_ANGLE = 0.6      # ~34 degrees - below limits to avoid saturation
        self.MIN_Z = 0.5          # minimum altitude
        self.MAX_Z = 10.0         # maximum altitude
        
        # Control mode
        self.control_active = True

    def update_from_keyboard(self):
        """
        Process keyboard input and update references
        
        Returns:
            (roll_ref, pitch_ref, yaw_ref, z_ref)
        """
        keys = pygame.key.get_pressed()
        
        if not self.control_active:
            return self.roll_ref, self.pitch_ref, self.yaw_ref, self.z_ref
        
        # ---- ALTITUDE CONTROL ----
        # SPACE: increase altitude (10 times faster than before!)
        if keys[pygame.K_SPACE]:
            self.z_ref = min(self.z_ref + self.alt_step * 10, self.MAX_Z)
        # SHIFT: decrease altitude (10 times faster!)
        elif keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
            self.z_ref = max(self.z_ref - self.alt_step * 10, self.MIN_Z)
        # No decay for altitude - keep reference
        
        # ---- PITCH CONTROL (forward/backward) ----
        # W: pitch forward
        if keys[pygame.K_w]:
            self.pitch_ref = min(self.pitch_ref + self.angle_step, self.MAX_ANGLE)
        # S: pitch backward
        elif keys[pygame.K_s]:
            self.pitch_ref = max(self.pitch_ref - self.angle_step, -self.MAX_ANGLE)
        else:
            # IMMEDIATE RESET - no decay, just go to 0
            self.pitch_ref = 0.0
        
        # ---- ROLL CONTROL (left/right) ----
        # A: roll left
        if keys[pygame.K_a]:
            self.roll_ref = min(self.roll_ref + self.angle_step, self.MAX_ANGLE)
        # D: roll right
        elif keys[pygame.K_d]:
            self.roll_ref = max(self.roll_ref - self.angle_step, -self.MAX_ANGLE)
        else:
            # IMMEDIATE RESET - no decay, just go to 0
            self.roll_ref = 0.0
        
        # ---- YAW CONTROL (rotation) ----
        # Q: rotate counter-clockwise
        if keys[pygame.K_q]:
            self.yaw_ref += self.angle_step
        # E: rotate clockwise
        elif keys[pygame.K_e]:
            self.yaw_ref -= self.angle_step
        else:
            # Reset yaw to 0
            self.yaw_ref = 0.0
        
        # Normalize yaw to [-pi, pi]
        while self.yaw_ref > math.pi:
            self.yaw_ref -= 2 * math.pi
        while self.yaw_ref < -math.pi:
            self.yaw_ref += 2 * math.pi
        
        return self.roll_ref, self.pitch_ref, self.yaw_ref, self.z_ref

    def reset(self):
        """Reset all references to zero/default"""
        self.z_ref = 2.5
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.yaw_ref = 0.0

    def toggle_control(self):
        """Toggle control active/inactive"""
        self.control_active = not self.control_active