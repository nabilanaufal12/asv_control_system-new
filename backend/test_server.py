# backend/test_server.py
# Server minimalis untuk debugging error 404.
# File ini tidak butuh file lain untuk berjalan.

from flask import Flask, jsonify, request

# Pastikan Anda sudah menginstal flask: pip install flask
app = Flask(__name__)

@app.route('/')
def index():
    """Halaman tes untuk browser."""
    return "<h1>Test Server is Running!</h1><p>Anda bisa menutup halaman ini.</p>"

@app.route('/status', methods=['GET'])
def get_status():
    """Endpoint status palsu."""
    print("✅ Endpoint /status berhasil diakses!")
    return jsonify({"status": "OK from Test Server"})

@app.route('/command', methods=['POST'])
def handle_command():
    """Endpoint command palsu."""
    print("✅✅✅ Endpoint /command berhasil diakses! ✅✅✅")
    data = request.json
    print(f"   -> Menerima data: {data}")
    return jsonify({"status": "command_received_by_test_server", "command": data.get("command")})

if __name__ == "__main__":
    print("🚀 Memulai Test Server untuk Debugging...")
    print("   -> Buka http://127.0.0.1:5000 di browser Anda untuk memastikan server berjalan.")
    print("   -> Server berjalan di http://0.0.0.0:5000")
    # debug=True akan memberikan output error yang lebih detail jika ada masalah
    app.run(host='0.0.0.0', port=5000, debug=True)
