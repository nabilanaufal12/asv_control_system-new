#!/bin/bash
# ==============================================================================
# Script Instalasi Auto-Start Service ASV Navantara KKI 2026
# ==============================================================================

set -e

SERVICE_FILE="/home/navantara/navantara/scripts/navantara.service"
TARGET_DEST="/etc/systemd/system/navantara.service"

echo "🚀 [Navantara Auto-Start Installer]"

if [ ! -f "$SERVICE_FILE" ]; then
    echo "❌ Error: File $SERVICE_FILE tidak ditemukan!"
    exit 1
fi

echo "📋 Menyalin service file ke $TARGET_DEST..."
sudo cp "$SERVICE_FILE" "$TARGET_DEST"
sudo chmod 644 "$TARGET_DEST"

echo "🔄 Memuat ulang konfigurasi systemd..."
sudo systemctl daemon-reload

echo "✅ Mengaktifkan auto-start saat boot..."
sudo systemctl enable navantara.service

echo ""
echo "🎉 Instalasi selesai! Perintah operasional yang berguna:"
echo "  - Menjalankan service sekarang : sudo systemctl start navantara.service"
echo "  - Memeriksa status service     : sudo systemctl status navantara.service"
echo "  - Menghentikan service         : sudo systemctl stop navantara.service"
echo "  - Melihat log real-time        : journalctl -u navantara.service -f"
echo "  - Mematikan auto-start         : sudo systemctl disable navantara.service"
