import eventlet
eventlet.monkey_patch() # Penting untuk thread latar belakang socketio

from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'rahasia!'
socketio = SocketIO(app, async_mode='eventlet')

def send_data_loop():
    """Kirim data counter setiap 1 detik."""
    count = 0
    while True:
        count += 1
        data = {'counter': count, 'message': 'Ini data dari server tes!'}
        
        # Cetak di terminal server
        print(f"Mengirim data: {data}")
        
        # Kirim ke SEMUA klien (termasuk Klien Web)
        socketio.emit('test_data', data)
        
        # Gunakan socketio.sleep() bukan time.sleep()
        socketio.sleep(1)

@app.route('/')
def index():
    """Sajikan file HTML klien."""
    # Asumsi 'test_client.html' ada di folder yang sama
    print("Mengirim file test_client.html ke browser.")
    return send_from_directory('.', 'test_client.html')

if __name__ == '__main__':
    print("Menjalankan server tes Socket.IO di http://0.0.0.0:5000")
    
    # Jalankan loop pengirim data di latar belakang
    socketio.start_background_task(send_data_loop)
    
    # Jalankan server Flask
    socketio.run(app, host='0.0.0.0', port=5000)