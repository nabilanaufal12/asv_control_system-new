# src/navantara_backend/core/gemini_commander.py
import google.generativeai as genai
import os

class GeminiCommander:
    def __init__(self, config):
        self.api_key = os.environ.get('GEMINI_API_KEY')
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        
        genai.configure(api_key=self.api_key)
        
        # Konfigurasi model Gemini Pro
        self.model = genai.GenerativeModel('gemini-pro')
        self.chat = self.model.start_chat(history=[])
        
        # Prompt sistem untuk memberikan konteks dan peran pada AI
        self.system_prompt = """
        Anda adalah AI Komandan untuk sebuah Autonomous Surface Vehicle (ASV) bernama NAVANTARA dalam sebuah kompetisi.
        Tugas Anda adalah membuat keputusan taktis berdasarkan data sensor dan deteksi visual.
        Output Anda HARUS berupa perintah JSON yang ringkas dan dapat dieksekusi.

        Data yang Anda terima berupa JSON berisi:
        - 'state': Data telemetri (posisi, heading, kecepatan).
        - 'detections': Daftar objek yang terdeteksi oleh kamera.

        Perintah yang bisa Anda keluarkan (pilih SATU):
        1. {"command": "CONTINUE_MISSION"} -> Lanjutkan navigasi waypoint normal.
        2. {"command": "SET_VIRTUAL_WAYPOINT", "payload": {"lat": float, "lon": float}} -> Perintahkan kapal untuk menuju waypoint sementara sebelum melanjutkan misi.
        3. {"command": "INVESTIGATE", "payload": {"object_id": int}} -> Fokus dan dekati objek spesifik untuk diinvestigasi.
        4. {"command": "AVOIDANCE_MANEUVER", "payload": {"direction": "PORT" | "STARBOARD"}} -> Lakukan manuver menghindar ke kiri (port) atau kanan (starboard).

        Analisis situasi dengan cermat. Prioritaskan keamanan dan penyelesaian misi sesuai aturan.
        Berikan hanya format JSON sebagai jawaban, tanpa penjelasan tambahan.
        """
        self._initialize_commander()
        print("[GeminiCommander] Komandan Misi AI siap.")

    def _initialize_commander(self):
        """Mengirim prompt sistem awal untuk mengatur konteks chat."""
        self.chat.send_message(self.system_prompt)

    def analyze_and_decide(self, asv_state, detections):
        """Menganalisis data dan menghasilkan keputusan taktis."""
        
        # Buat prompt input dari data real-time
        input_data = {
            "state": asv_state,
            "detections": detections
        }
        
        prompt = f"Data sensor saat ini: {input_data}. Apa perintah Anda?"
        
        try:
            response = self.chat.send_message(prompt)
            # Membersihkan output untuk memastikan hanya JSON yang valid
            cleaned_response = response.text.strip().replace('```json', '').replace('```', '')
            return cleaned_response
        except Exception as e:
            print(f"[GeminiCommander] Error saat berkomunikasi dengan API: {e}")
            # Jika gagal, berikan perintah aman default
            return '{"command": "CONTINUE_MISSION"}'