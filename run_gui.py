# run_gui.py
import sys
import os

# Menambahkan path src ke sys.path agar impor dari backend dan gui berhasil
try:
    project_root = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(project_root, 'src')
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
except NameError:
    # Fallback jika dijalankan di lingkungan yang tidak memiliki __file__
    sys.path.insert(0, "./src")

from navantara_gui import main_gui

if __name__ == "__main__":
    main_gui.run()