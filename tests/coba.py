from flask import Flask, Response, stream_with_context
from flask_cors import CORS  # 1. Import library CORS
import time
import json
import random

app = Flask(__name__)
# 2. Terapkan CORS ke seluruh aplikasi.
# Ini akan otomatis menambahkan header 'Access-Control-Allow-Origin: *'
CORS(app)


@app.route("/data")
def get_data():
    """Endpoint ini menggunakan Server-Sent Events (SSE) untuk streaming data."""

    def event_stream():
        count = 0
        while True:
            # 3. Buat data dummy (ganti ini dengan data sensor Anda)
            data_sensor = {
                "id": count,
                "suhu": round(random.uniform(25.0, 30.0), 2),
                "kelembaban": round(random.uniform(60.0, 80.0), 2),
            }

            # 4. Format data untuk SSE: "data: ...\n\n"
            json_data = json.dumps(data_sensor)
            yield f"data: {json_data}\n\n"

            count += 1
            time.sleep(2)  # Kirim data setiap 2 detik

    # Menggunakan stream_with_context agar lebih efisien
    return Response(
        stream_with_context(event_stream()), content_type="text/event-stream"
    )


if __name__ == "__main__":
    # Penting: Gunakan host='0.0.0.0' agar server bisa diakses dari IP 192.168.1.8
    # Jangan gunakan '127.0.0.1' atau 'localhost'
    app.run(debug=True, host="0.0.0.0", port=5000, threaded=True)
