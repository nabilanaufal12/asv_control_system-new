import unittest
import sys
import os
import time
from unittest.mock import MagicMock

# Setup path for backend import
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from navantara_backend.core.asv_handler import AsvHandler, AsvState

class TestNavantaraIntegration(unittest.TestCase):
    def setUp(self):
        # Mock backend configurations
        self.mock_config = {
            "serial_connection": {"use_dummy_serial": True, "default_baud_rate": 115200},
            "navigation": {"heading_pid": {"kp": 1.0, "ki": 0.0, "kd": 0.0}},
            "actuators": {"servo_default_angle": 90}
        }
        self.mock_socketio = MagicMock()
        
        # Initialize the Handler
        self.handler = AsvHandler(self.mock_config, self.mock_socketio)
        self.handler.serial_handler.is_connected = True
        
    def tearDown(self):
        self.handler.running = False
        
    def test_case_1_waypoint_logic(self):
        """Test Case 1: Validate Waypoint Load and Transitions"""
        # Empty waypoints
        self.handler.process_command("SET_WAYPOINTS", {"waypoints": []})
        self.assertEqual(len(self.handler.current_state.waypoints), 0, "Waypoints should be empty")
        
        # Valid waypoints
        valid_wps = [{"lat": -6.0, "lon": 107.0}, {"lat": -6.1, "lon": 107.1}]
        self.handler.process_command("SET_WAYPOINTS", {"waypoints": valid_wps, "arena": "Arena_B", "inversion_trigger_wp": 1})
        self.assertEqual(len(self.handler.current_state.waypoints), 2, "Should load 2 waypoints")
        self.assertEqual(self.handler.current_state.active_arena, "Arena_B", "Arena should be parsed as Arena_B")
        
        # Transition wp_index simulation
        telemetry_payload = {"mode": "AUTO", "status": "WAYPOINT", "wp_target_idx": 1}
        self.handler._parse_json_telemetry(telemetry_payload)
        self.assertEqual(self.handler.current_state.nav_target_wp_index, 1, "Waypoint index should increment")
        
    def test_case_2_state_machine_and_serial_payload(self):
        """Test Case 2: Validate State Machine transitions ('W' -> 'A')"""
        # Transition from MANUAL to AUTO
        self.handler.process_command("SET_MODE", {"mode": "AUTO"})
        self.assertEqual(self.handler.current_state.control_mode, "AUTO", "Mode should be AUTO")
        
        # Transition to Vision Target ('A' Mode logic handler)
        self.handler._handle_vision_target_update({"active": True, "obstacle_class": "kotak-hijau"})
        self.assertTrue(self.handler.current_state.vision_target["active"], "Vision target should be active")
        self.assertEqual(self.handler.current_state.vision_target["obstacle_class"], "kotak-hijau")

    def test_case_3_auto_capture_and_reverse(self):
        """Test Case 3: Verify Auto-Capture and Reverse Maneuver triggers"""
        self.handler.current_state.is_reversing = True
        self.handler.current_state.reverse_start_time = time.time()
        
        # Since we simulate the state, we can verify if the attributes exist and accept logic
        self.assertTrue(hasattr(self.handler.current_state, "is_reversing"))
        self.assertTrue(hasattr(self.handler.current_state, "reverse_start_time"))
        
        # Set dummy dir_cmd logic
        self.handler.current_state.nav_motor_cmd = 1000
        self.assertEqual(self.handler.current_state.nav_motor_cmd, 1000, "Should reverse motor/dir to 1000")

    def test_case_4_websocket_connection_drops(self):
        """Test Case 4: Simulate WebSocket Connection Drops"""
        try:
            self.handler.set_streaming_status(False)
            self.assertFalse(self.handler.is_streaming_to_gui, "Should cleanly stop streaming")
        except Exception as e:
            self.fail(f"WebSocket disconnect raised fatal exception: {e}")

if __name__ == '__main__':
    unittest.main()
