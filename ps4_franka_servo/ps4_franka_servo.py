#!/usr/bin/env python3
"""
UNIFIED PS4 Franka Control Node
Combines arm movement (via MoveIt Servo) and gripper control with PROPER Franka actions

Author: Diana
Integrated with franka_msgs actions for reliable gripper control
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import GraspEpsilon
import threading

class UnifiedPS4FrankaControl(Node):
    """
    Unified controller for PS4-based Franka Panda arm and gripper control.
    """
    
    def __init__(self):
        super().__init__('ps4_franka_servo')
        
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )
        
        self.servo_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        self.servo_client = self.create_client(
            ServoCommandType, 
            '/servo_node/switch_command_type'
        )
        
        self.gripper_move_client = ActionClient(self, Move, '/panda_gripper/move')
        self.gripper_grasp_client = ActionClient(self, Grasp, '/panda_gripper/grasp')
        
        self.linear_scale = 0.1
        self.angular_scale = 0.6
        self.dead_zone = 0.15
        
        self.gripper_speed = 0.05
        self.gripper_width = 0.08
        self.gripper_force = 1.0

        self.servo_configured = False
        self.config_attempted = False
        self.control_mode = 0
        self.mode_changed = False
        
        self.gripper_is_open = True
        self.gripper_busy = False
        self.last_square_state = 0
        
        self.last_button_states = [0] * 13
        
        self.get_logger().info("PS4 Franka Servo Controller started")
        self.get_logger().info("Arm Controls:")
        self.get_logger().info("  - CIRCLE (O): Activate Servo mode")
        self.get_logger().info("  - L1: Switch between Translation/Rotation modes")
        self.get_logger().info("Gripper Control:")
        self.get_logger().info("  - SQUARE: Toggle gripper open/close")
        self.get_logger().info("Current mode: TRANSLATION | Gripper: OPEN")
        
        self.connect_to_gripper_servers()
    
    def connect_to_gripper_servers(self):
        """Attempt to connect to gripper action servers"""
        self.get_logger().info("Connecting to gripper action servers")
        
        thread = threading.Thread(target=self._connect_gripper_background)
        thread.daemon = True
        thread.start()
    
    def _connect_gripper_background(self):
        """Background thread for gripper server connection"""
        import time
        
        if self.gripper_move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Connected to /panda_gripper/move")
        else:
            self.get_logger().warning("/panda_gripper/move not available yet")
        
        if self.gripper_grasp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Connected to /panda_gripper/grasp")
        else:
            self.get_logger().warning("/panda_gripper/grasp not available yet")
    
    def configure_servo(self):
        """Configure MoveIt Servo to accept twist commands"""
        if self.config_attempted:
            return
            
        self.config_attempted = True
        self.get_logger().info("Configuring Servo for twist commands")
        
        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servo service not available!")
            return
            
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        
        future = self.servo_client.call_async(request)
        future.add_done_callback(self.servo_config_callback)
    
    def servo_config_callback(self, future):
        """Handle the response from Servo configuration service call"""
        try:
            response = future.result()
            if response.success:
                self.servo_configured = True
                self.get_logger().info("Servo configured for twist commands")
            else:
                self.get_logger().error("Servo configuration failed")
        except Exception as e:
            self.get_logger().error(f"Servo configuration error: {e}")
    
    def apply_dead_zone(self, value):
        """Apply dead zone to controller input to prevent drift"""
        if abs(value) < self.dead_zone:
            return 0.0
        else:
            return (value - self.dead_zone * (1.0 if value > 0 else -1.0)) / (1.0 - self.dead_zone)
    
    def toggle_gripper(self):
        """Toggle gripper between open and closed states"""
        if self.gripper_busy:
            self.get_logger().info("Wait, gripper is moving")
            return
        
        self.gripper_busy = True
        
        if self.gripper_is_open:
            action = "CLOSE"
            self.get_logger().info(f"Gripper: {action}")
            thread = threading.Thread(target=self._close_gripper)
        else:
            action = "OPEN"
            self.get_logger().info(f"Gripper: {action}")
            thread = threading.Thread(target=self._open_gripper)
        
        thread.daemon = True
        thread.start()
    
    def _open_gripper(self):
        """Open gripper in background thread"""
        try:
            if not self.gripper_move_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Cannot connect to gripper move server")
                self.gripper_busy = False
                return
            
            goal = Move.Goal(width=self.gripper_width, speed=self.gripper_speed)
            future = self.gripper_move_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                goal_handle = future.result()
                if goal_handle:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                    if result_future.done():
                        result = result_future.result().result
                        if result.success:
                            self.gripper_is_open = True
                            self.get_logger().info("Gripper opened successfully")
                        else:
                            self.get_logger().warning("Gripper open command may have failed")
                else:
                    self.get_logger().error("Gripper open goal was rejected")
            else:
                self.get_logger().warning("Gripper open command timeout")
                
        except Exception as e:
            self.get_logger().error(f"Gripper open error: {e}")
        finally:
            self.gripper_busy = False
    
    def _close_gripper(self):
        """Close gripper in background thread"""
        try:
            if not self.gripper_grasp_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Cannot connect to gripper grasp server")
                self.gripper_busy = False
                return
            
            goal = Grasp.Goal(
                width=0.0,
                speed=self.gripper_speed,
                force=self.gripper_force,
                epsilon=GraspEpsilon(inner=float('inf'), outer=float('inf'))
            )
            
            future = self.gripper_grasp_client.send_goal_async(goal)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                goal_handle = future.result()
                if goal_handle:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                    if result_future.done():
                        result = result_future.result().result
                        if result.success:
                            self.gripper_is_open = False
                            self.get_logger().info("Gripper closed successfully")
                        else:
                            self.get_logger().warning("Gripper close command may have failed")
                else:
                    self.get_logger().error("Gripper close goal was rejected")
            else:
                self.get_logger().warning("Gripper close command timeout")
                
        except Exception as e:
            self.get_logger().error(f"Gripper close error: {e}")
        finally:
            self.gripper_busy = False
    
    def joy_callback(self, msg):
        """
        Process incoming PS4 controller messages and generate robot commands
        """
        if len(msg.buttons) < 13:
            return

        # CIRCLE button (index 1): Activate Servo
        if msg.buttons[1] == 1 and self.last_button_states[1] == 0 and not self.config_attempted:
            self.configure_servo()
        
        if msg.buttons[4] == 1 and self.last_button_states[4] == 0 and not self.mode_changed:
            self.mode_changed = True
            self.control_mode = 1 - self.control_mode
            mode_name = "ROTATION" if self.control_mode == 1 else "TRANSLATION"
            self.get_logger().info(f"Control mode changed to: {mode_name}")
        elif msg.buttons[4] == 0:
            self.mode_changed = False
        
        if msg.buttons[3] == 1 and self.last_button_states[3] == 0:
            self.toggle_gripper()
        
        for i in range(min(len(msg.buttons), len(self.last_button_states))):
            self.last_button_states[i] = msg.buttons[i]

        if not self.servo_configured:
            if not self.config_attempted:
                self.get_logger().info("Press CIRCLE (X) to activate Servo mode", once=True)
            return
        
        if len(msg.axes) < 6:
            return
            
        left_stick_x = msg.axes[1]
        left_stick_y = msg.axes[0]
        l2_trigger = msg.axes[2]
        right_stick_x = msg.axes[3]
        r2_trigger = msg.axes[5]
        
        left_stick_x = self.apply_dead_zone(left_stick_x)
        left_stick_y = self.apply_dead_zone(left_stick_y)
        right_stick_x = self.apply_dead_zone(right_stick_x)
        
        l2_trigger = (1.0 - l2_trigger) / 2.0
        r2_trigger = (1.0 - r2_trigger) / 2.0
        
        l2_trigger = 0.0 if l2_trigger < self.dead_zone else l2_trigger
        r2_trigger = 0.0 if r2_trigger < self.dead_zone else r2_trigger
        
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "panda_link0"
        
        if self.control_mode == 0:
            twist_msg.twist.linear.x = left_stick_x * self.linear_scale
            twist_msg.twist.linear.y = left_stick_y * self.linear_scale
            z_control = r2_trigger - l2_trigger
            twist_msg.twist.linear.z = z_control * self.linear_scale
            twist_msg.twist.angular.z = right_stick_x * self.angular_scale
        else:
            twist_msg.twist.angular.y = left_stick_x * self.angular_scale
            twist_msg.twist.angular.x = -left_stick_y * self.angular_scale
            z_control = r2_trigger - l2_trigger
            twist_msg.twist.linear.z = z_control * self.linear_scale
            twist_msg.twist.angular.z = right_stick_x * self.angular_scale
        
        self.servo_pub.publish(twist_msg)

def main():
    """Main function to initialize and run the unified PS4 Franka control node"""
    rclpy.init()
    node = UnifiedPS4FrankaControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PS4 Franka servo controller shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
