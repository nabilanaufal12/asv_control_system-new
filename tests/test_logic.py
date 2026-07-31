import sys
import os

sys.path.insert(0, os.path.abspath("src"))
from navantara_backend.core.asv_handler import AsvHandler  # noqa: E402
from flask_socketio import SocketIO  # noqa: E402


class DummyApp:
    config = {"ASV_CONFIG": {}}


app = DummyApp()
handler = AsvHandler(app.config["ASV_CONFIG"], SocketIO())
handler.current_state.debug_mode_enabled = True
handler._handle_debug_command({"set_wp_index": 12})
handler._handle_debug_command({"set_dist_to_wp": 2.0})

# run a fake main logic loop
handler.current_state.control_mode = "AUTO"
handler.main_logic_loop()
