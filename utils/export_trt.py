import os
from ultralytics import YOLO

def export_tensorrt():
    print("="*60)
    print(" YOLOv11 TensorRT (FP16) Engine Export Tool")
    print("="*60)
    
    # Path to the PyTorch model
    model_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'navantara_backend', 'vision', 'best.pt'))
    
    if not os.path.exists(model_path):
        print(f"[ERROR] PyTorch model not found at: {model_path}")
        return
        
    print(f"[1] Loading PyTorch model from: {model_path}")
    model = YOLO(model_path)
    
    print("\n[2] Starting TensorRT Export Process...")
    print("    - Format: engine (TensorRT)")
    print("    - Device: GPU 0 (Jetson Orin Nano)")
    print("    - Precision: FP16 (half=True)")
    print("    - Workspace: 4GB VRAM")
    print("\n[!] WARNING: This native optimization process is highly intensive.")
    print("             It may take 10 - 20 minutes to complete depending on thermal constraints.")
    print("             DO NOT interrupt the process!\n")
    
    # Perform the export
    exported_path = model.export(
        format='engine',
        device=0,
        half=True,
        workspace=4
    )
    
    print("\n" + "="*60)
    print(f"[3] SUCCESS! Model exported to TensorRT engine.")
    print(f"    Engine file located at: {exported_path}")
    print("==================================================")

if __name__ == '__main__':
    export_tensorrt()
