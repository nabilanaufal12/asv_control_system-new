import cv2
from ultralytics import YOLO

# Tambahkan ini di vision_service.py Anda nanti
CLASS_NAMES = {
    0: 'bola-biru',
    1: 'kotak-biru',
    2: 'bola-hijau',
    3: 'kotak-hijau',
    4: 'bola-merah'
}

def main():
    print("Memuat model best.pt (tanpa TensorRT, menggunakan PyTorch)...")
    
    try:
        # Memuat model YOLO
        model = YOLO("asv_final_v11.engine", task='detect')
    except Exception as e:
        print(f"Error memuat model: {e}")
        print("Pastikan model best.pt ada di folder yang sama.")
        return

    print("Membuka kamera (webcam 0)... Tekan 'q' untuk keluar.")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Tidak dapat membuka kamera.")
        print("Silakan coba mengubah index VideoCapture(0) jika Anda memiliki kamera eksternal.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal mengambil frame dari kamera.")
            break
            
        # Jalankan inferensi menggunakan model asv_final_v11.engine
        # conf=0.5 berarti hanya deteksi dengan confidence >= 50% yang ditampilkan
        results = model.predict(source=frame, conf=0.5, verbose=False)
        
        # Iterasi hasil deteksi untuk mengganti nama class dengan CLASS_NAMES kustom jika diperlukan,
        # namun results[0].plot() akan otomatis menggambar kotak dengan nama class bawaan model.
        # Kita juga bisa menggambar secara manual:
        
        annotated_frame = frame.copy()
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Koordinat bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Class ID dan confidence
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                
                # Dapatkan nama class dari dictionary CLASS_NAMES kita (jika ada), kalau tidak gunakan default
                label_name = model.names[cls_id]
                label = f"{label_name} {conf:.2f}"
                
                # Gambar kotak
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Gambar label
                cv2.putText(annotated_frame, label, (x1, max(0, y1 - 10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Tampilkan hasil inferensi
        cv2.imshow("YOLO Inference - best.pt", annotated_frame)
        
        # Tekan tombol 'q' untuk keluar
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
