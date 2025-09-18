# navantara_backend/vision/inference_engine.py
# --- VERSI DENGAN PATH MODEL YANG ROBUST DAN WARNA BOUNDING BOX DINAMIS ---

import torch
import numpy as np
import traceback
import cv2
from pathlib import Path
import os
import sys


# --- BLOK BARU: Pencarian Path yang Dinamis ---
# Mencari direktori root proyek (yang berisi folder 'src') secara dinamis
def find_project_root(marker_folder="src"):
    current_path = Path(__file__).resolve()
    for parent in current_path.parents:
        if (parent / marker_folder).is_dir():
            return parent
    # Fallback jika struktur tidak terduga, gunakan direktori kerja saat ini
    print(
        f"[Engine] PERINGATAN: Tidak dapat menemukan root proyek (folder '{marker_folder}'). Menggunakan direktori kerja saat ini."
    )
    return Path.cwd()


PROJECT_ROOT = find_project_root()
YOLOV5_PATH = PROJECT_ROOT / "src" / "navantara_backend" / "yolov5"

# Menambahkan path yolov5 ke sys.path jika belum ada, penting untuk torch.hub.load
if str(YOLOV5_PATH) not in sys.path:
    sys.path.insert(0, str(YOLOV5_PATH))
# --- AKHIR BLOK BARU ---


# Cek ketersediaan TensorRT secara aman
try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit

    TENSORRT_AVAILABLE = True
except ImportError:
    TENSORRT_AVAILABLE = False

# =================================================================================
# HELPER CLASS INTERNAL UNTUK TENSORRT (Hanya digunakan jika tersedia)
# =================================================================================
if TENSORRT_AVAILABLE:

    class TRTInference:
        """Kelas internal untuk menangani interaksi tingkat rendah dengan engine TensorRT."""

        def __init__(self, engine_path):
            self.logger = trt.Logger(trt.Logger.WARNING)
            with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
                self.engine = runtime.deserialize_cuda_engine(f.read())

            self.context = self.engine.create_execution_context()
            self.inputs, self.outputs, self.bindings, self.stream = (
                self._allocate_buffers()
            )

            input_binding_name = self.engine.get_binding_name(0)
            self.input_shape = self.engine.get_binding_shape(input_binding_name)

        def _allocate_buffers(self):
            inputs, outputs, bindings = [], [], []
            stream = cuda.Stream()
            for i in range(self.engine.num_bindings):
                binding_name = self.engine.get_binding_name(i)
                size = trt.volume(self.engine.get_binding_shape(binding_name))
                dtype = trt.nptype(self.engine.get_binding_dtype(binding_name))

                host_mem = cuda.pagelocked_empty(size, dtype)
                device_mem = cuda.mem_alloc(host_mem.nbytes)
                bindings.append(int(device_mem))

                if self.engine.binding_is_input(binding_name):
                    inputs.append({"host": host_mem, "device": device_mem})
                else:
                    outputs.append({"host": host_mem, "device": device_mem})
            return inputs, outputs, bindings, stream

        def infer(self, image_preprocessed):
            np.copyto(self.inputs[0]["host"], image_preprocessed.ravel())
            cuda.memcpy_htod_async(
                self.inputs[0]["device"], self.inputs[0]["host"], self.stream
            )
            self.context.execute_async_v2(
                bindings=self.bindings, stream_handle=self.stream.handle
            )
            cuda.memcpy_dtoh_async(
                self.outputs[0]["host"], self.outputs[0]["device"], self.stream
            )
            self.stream.synchronize()
            return self.outputs[0]["host"]


# =================================================================================
# KELAS UTAMA YANG DIGUNAKAN OLEH VISION SERVICE
# =================================================================================
class InferenceEngine:
    """
    Kelas utama yang menangani semua logika inferensi.
    Secara otomatis memilih backend terbaik yang tersedia (TensorRT atau PyTorch).
    """

    def __init__(self, config):
        self.config = config
        self.yolov5_path = YOLOV5_PATH
        self.model = None
        self.use_tensorrt = TENSORRT_AVAILABLE

        vision_cfg = self.config.get("vision", {})
        self.conf_thresh = float(vision_cfg.get("conf_threshold", 0.25))
        self.iou_thresh = float(vision_cfg.get("iou_threshold", 0.45))
        self.class_names = vision_cfg.get("class_names", [])

        # --- PERUBAHAN DI SINI: Tambahkan warna untuk kotak ---
        self.color_palette = {
            "red_buoy": (51, 51, 204),  # BGR untuk Merah (Pylox 33)
            "green_buoy": (0, 150, 0),  # BGR untuk Hijau (Pylox 105)
            "green_box": (0, 255, 0),  # BGR untuk Hijau Terang
            "blue_box": (255, 0, 0),  # BGR untuk Biru
            "default": (0, 255, 255),  # BGR untuk Kuning (sebagai default)
        }
        # --- AKHIR PERUBAHAN ---

        self._initialize_model()

    def _initialize_model(self):
        """Mencoba memuat TensorRT, jika gagal akan fallback ke PyTorch."""
        if self.use_tensorrt:
            try:
                engine_path = self.yolov5_path / "besto.engine"
                if engine_path.exists():
                    print("[Engine] Mencoba memuat model TensorRT...")
                    self.model = TRTInference(str(engine_path))
                    print(
                        f"[Engine] Model TensorRT berhasil dimuat. Input shape: {self.model.input_shape}"
                    )
                    return
                else:
                    print(
                        "[Engine] PERINGATAN: File 'besto.engine' tidak ditemukan. Beralih ke PyTorch."
                    )
                    self.use_tensorrt = False
            except Exception as e:
                print(
                    f"[Engine] GAGAL memuat engine TensorRT: {e}. Beralih ke PyTorch."
                )
                traceback.print_exc()
                self.use_tensorrt = False

        print("[Engine] Menggunakan mode fallback PyTorch.")
        try:
            weights_path = self.yolov5_path / "besto.pt"
            if not weights_path.exists():
                print(f"[Engine] KRITIS: File bobot '{weights_path}' tidak ditemukan.")
                self.model = None
                return

            self.model = torch.hub.load(
                str(self.yolov5_path),
                "custom",
                path=str(weights_path),
                source="local",
                force_reload=True,
                trust_repo=True,
            )
            self.model.conf = self.conf_thresh
            self.model.iou = self.iou_thresh
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model.to(device)
            print(f"[Engine] Model PyTorch berhasil dimuat di device {device}.")
        except Exception as e:
            print("[Engine] KRITIS: Gagal memuat model PyTorch.")
            traceback.print_exc()
            self.model = None

    def infer(self, frame):
        """
        Melakukan inferensi pada sebuah frame. Metode ini menangani semua
        langkah pre-processing, inferensi, dan post-processing.
        """
        if self.model is None:
            return [], frame

        if self.use_tensorrt:
            preprocessed_image, ratio, (dw, dh) = self._preprocess_trt(frame)
            raw_output = self.model.infer(preprocessed_image)
            detections = self._postprocess_trt(raw_output, ratio, (dw, dh))
            annotated_frame = self._annotate_frame(frame, detections)
            return detections, annotated_frame
        else:
            # Untuk PyTorch, kita perlu memodifikasi cara rendering
            results = self.model(frame)
            detections = self._pandas_to_dict(results.pandas().xyxy[0])
            # Gambar anotasi secara manual alih-alih menggunakan results.render()
            annotated_frame = self._annotate_frame(frame.copy(), detections)
            return detections, annotated_frame

    def _preprocess_trt(self, img):
        """Pre-process gambar untuk input TensorRT: resize, letterbox, normalize."""
        input_h, input_w = self.model.input_shape[2], self.model.input_shape[3]
        h, w, _ = img.shape

        ratio = min(input_w / w, input_h / h)
        new_w, new_h = int(w * ratio), int(h * ratio)
        dw, dh = (input_w - new_w) / 2, (input_h - new_h) / 2

        resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        padded_img = cv2.copyMakeBorder(
            resized_img,
            int(dh),
            int(dh) + (input_h - new_h) % 2,
            int(dw),
            int(dw) + (input_w - new_w) % 2,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        blob = padded_img.transpose(2, 0, 1)
        blob = np.ascontiguousarray(blob, dtype=np.float32)
        blob /= 255.0
        return blob, ratio, (dw, dh)

    def _postprocess_trt(self, output, ratio, pad):
        """Post-process output mentah dari TensorRT, termasuk NMS."""
        num_boxes = output.shape[0] // (5 + len(self.class_names))
        output = output.reshape(1, num_boxes, 5 + len(self.class_names))

        boxes = []
        for det in output[0]:
            confidence = det[4]
            if confidence < self.conf_thresh:
                continue

            class_scores = det[5:]
            class_id = np.argmax(class_scores)
            max_score = class_scores[class_id]

            if max_score * confidence < self.conf_thresh:
                continue

            cx, cy, w, h = det[0:4]
            x1 = (cx - w / 2 - pad[0]) / ratio
            y1 = (cy - h / 2 - pad[1]) / ratio
            x2 = (cx + w / 2 - pad[0]) / ratio
            y2 = (cy + h / 2 - pad[1]) / ratio

            boxes.append([x1, y1, x2, y2, confidence * max_score, class_id])

        final_boxes = self._non_max_suppression(np.array(boxes))

        detections = []
        for box in final_boxes:
            x1, y1, x2, y2, conf, class_id = box
            detections.append(
                {
                    "xyxy": [x1, y1, x2, y2],
                    "center": (int((x1 + x2) / 2), int((y1 + y2) / 2)),
                    "class": self.class_names[int(class_id)],
                    "confidence": conf,
                }
            )
        return detections

    def _non_max_suppression(self, boxes):
        """Implementasi sederhana dari Non-Maximum Suppression."""
        if len(boxes) == 0:
            return []

        boxes = boxes[boxes[:, 4].argsort()[::-1]]

        keep = []
        while len(boxes) > 0:
            best_box = boxes[0]
            keep.append(best_box)

            xA = np.maximum(best_box[0], boxes[1:, 0])
            yA = np.maximum(best_box[1], boxes[1:, 1])
            xB = np.minimum(best_box[2], boxes[1:, 2])
            yB = np.minimum(best_box[3], boxes[1:, 3])

            inter_area = np.maximum(0, xB - xA) * np.maximum(0, yB - yA)
            boxA_area = (best_box[2] - best_box[0]) * (best_box[3] - best_box[1])
            boxB_area = (boxes[1:, 2] - boxes[1:, 0]) * (boxes[1:, 3] - boxes[1:, 1])

            iou = inter_area / (boxA_area + boxB_area - inter_area)

            boxes = boxes[1:][iou < self.iou_thresh]

        return np.array(keep)

    def _annotate_frame(self, frame, detections):
        """Menggambar bounding box pada frame dengan warna dinamis."""
        annotated_frame = frame
        for det in detections:
            x1, y1, x2, y2 = map(int, det["xyxy"])

            # Tentukan warna berdasarkan kelas deteksi
            box_color = self.color_palette.get(
                det["class"], self.color_palette["default"]
            )

            label = f"{det['class']} {det['confidence']:.2f}"

            # Gambar persegi panjang (bounding box)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)

            # Gambar label teks
            cv2.putText(
                annotated_frame,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                box_color,
                2,
            )
        return annotated_frame

    def _pandas_to_dict(self, df):
        """Konversi output pandas dari model PyTorch ke format standar."""
        detections = []
        for _, row in df.iterrows():
            detections.append(
                {
                    "xyxy": [row["xmin"], row["ymin"], row["xmax"], row["ymax"]],
                    "center": (
                        int((row["xmin"] + row["xmax"]) / 2),
                        int((row["ymin"] + row["ymax"]) / 2),
                    ),
                    "class": row["name"],
                    "confidence": row["confidence"],
                }
            )
        return detections
