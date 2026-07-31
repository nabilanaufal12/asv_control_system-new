import os
import sys
import time
import json
import unittest
from unittest.mock import MagicMock, patch

# Ensure the src directory is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))

from navantara_backend.core.asv_handler import AsvHandler  # noqa: E402


class TestDockingLogic(unittest.TestCase):
    def setUp(self):
        # Create a dummy config
        self.test_config_path = "/tmp/test_config.json"
        self.config_data = {
            "docking_mission": {
                "enabled": True,
                "trigger_wp_index": 17,
                "target_arena": "Arena_A",
                "Arena_A": {
                    "servo_angle": 45,
                    "duration_sec": 20,
                    "rear_pwm": 1100,
                    "rear_dir": 2000,
                    "front_left_pwm": 1200,
                    "front_left_dir": 2000,
                    "front_right_pwm": 1200,
                    "front_right_dir": 1000,
                },
            }
        }
        with open(self.test_config_path, "w") as f:
            json.dump(self.config_data, f)

        # Mock SerialHandler and MissionLogger to avoid hardware dependencies
        with patch("navantara_backend.core.asv_handler.SerialHandler") as _, patch(
            "navantara_backend.core.asv_handler.MissionLogger"
        ) as _:

            self.handler = AsvHandler(config=self.config_data, socketio=MagicMock())
            # Replace the serial_handler with our mock
            self.handler.serial_handler = MagicMock()
            self.handler.serial_handler.is_connected = True

    def test_1_config_read_write(self):
        print("\n--- Test 1: Config Read/Write ---")
        # Simulate GUI update via our new hot-reload handler
        payload = {
            "Arena_A": {
                "servo_angle": 45,
                "duration_sec": 25,  # Updated to 25
                "rear_pwm": 1100,
                "rear_dir": 2000,
                "front_left_pwm": 1200,
                "front_left_dir": 2000,
                "front_right_pwm": 1200,
                "front_right_dir": 1000,
            }
        }
        self.handler._handle_update_docking_config(payload)

        # Verify it updated in memory
        self.assertEqual(
            self.handler.config["docking_mission"]["Arena_A"]["duration_sec"], 25
        )
        print("Config Hot-Reload successful. Duration is now 25s.")

    @patch("time.time")
    def test_2_and_3_docking_trigger_and_payload(self, mock_time):
        print("\n--- Test 2 & 3: Docking Trigger & Payload Verification ---")
        mock_time.return_value = 100.0  # Set a fixed start time

        # Mock ASV state to trigger docking
        self.handler.current_state.control_mode = "AUTO"
        self.handler.current_state.current_waypoint_index = 17  # trigger wp is 17
        self.handler.current_state.nav_dist_to_wp = 2.5  # < 3.0m
        self.handler.current_state.is_docking = False

        # Run logic loop in a thread for a fraction of a second
        import threading

        t = threading.Thread(target=self.handler.main_logic_loop)
        t.start()
        time.sleep(0.1)
        self.handler.running = False
        t.join()

        # Assert docking was triggered
        self.assertTrue(self.handler.current_state.is_docking)
        self.assertEqual(self.handler.current_state.docking_start_time, 100.0)
        print("Docking successfully triggered at WP 17 with distance 2.5m.")

        # Intercept the sent command
        calls = self.handler.serial_handler.send_command.call_args_list
        # Get the last command sent
        last_command = calls[-1][0][0]
        print(f"Sent Serial Command: {last_command.strip()}")

        # Verify 8-parameter format
        # Format: A,servo,dir_b,pwm_b,dir_fl,pwm_fl,dir_fr,pwm_fr\n
        parts = last_command.strip().split(",")
        self.assertEqual(len(parts), 8)
        self.assertEqual(parts[0], "A")
        self.assertEqual(parts[1], "45")  # servo
        self.assertEqual(parts[2], "2000")  # rear dir
        self.assertEqual(parts[3], "1100")  # rear pwm
        self.assertEqual(parts[4], "2000")  # fl dir
        self.assertEqual(parts[5], "1200")  # fl pwm
        self.assertEqual(parts[6], "1000")  # fr dir
        self.assertEqual(parts[7], "1200")  # fr pwm
        print("Payload successfully verified. Format is exactly 8 parameters.")

    @patch("time.time")
    def test_4_timer_and_abort(self, mock_time):
        print("\n--- Test 4: Timer & Abort ---")
        # Fast forward time by 21 seconds (duration is 20s)
        self.handler.current_state.control_mode = "AUTO"
        self.handler.current_state.is_docking = True
        self.handler.current_state.docking_start_time = 100.0

        mock_time.return_value = 121.0

        # Run logic loop in a thread for a fraction of a second
        self.handler.running = True
        import threading

        t = threading.Thread(target=self.handler.main_logic_loop)
        t.start()
        time.sleep(0.1)
        self.handler.running = False
        t.join()

        # Assert docking was aborted and mode switched to MANUAL
        self.assertFalse(self.handler.current_state.is_docking)
        self.assertEqual(self.handler.current_state.control_mode, "MANUAL")
        print("Docking properly timed out after 20s and switched to MANUAL.")

        # Verify the abort payload (STOP command)
        calls = self.handler.serial_handler.send_command.call_args_list
        last_command = calls[-1][0][0]
        print(f"Sent Abort Command: {last_command.strip()}")

        # A,90,2000,1000,2000,1000,2000,1000
        parts = last_command.strip().split(",")
        self.assertEqual(parts[0], "A")
        self.assertEqual(parts[1], "90")
        self.assertEqual(parts[3], "1000")  # rear pwm
        self.assertEqual(parts[5], "1000")  # fl pwm
        self.assertEqual(parts[7], "1000")  # fr pwm
        print("Abort Payload verified. All PWMs are safely at 1000.")


if __name__ == "__main__":
    unittest.main(verbosity=2)
