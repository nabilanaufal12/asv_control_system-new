// js/main.js

// Variabel global untuk melacak state
let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 };

document.addEventListener("DOMContentLoaded", () => {
  const CONFIG = {
    // Firebase tidak lagi digunakan
    // FIREBASE_URL: "...", 
    SUPABASE_URL: "https://ngmicoyombtgtlegdjzn.supabase.co",
    SUPABASE_BUCKET: "navantara",
    SUPABASE_TOKEN:
      "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im5nbWljb3lvbWJ0Z3RsZWdkanpuIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTM5NjkwOTMsImV4cCI6MjA2OTU0NTA5M30.tz7_mfVaX_5sWO3xtSQ1h85JAIRrL6iC4ZNm0TaUTdI",
  };

  const ELEMENTS = {
    hdgValue: document.getElementById("hdg-value"),
    sogValue: document.getElementById("sog-value"),
    cogValue: document.getElementById("cog-value"),
    dayValue: document.getElementById("day-value"),
    dateValue: document.getElementById("date-value"),
    gpsValue: document.getElementById("gps-value"),
    surfaceCaptures: document.getElementById("surface-captures"),
    underwaterCaptures: document.getElementById("underwater-captures"),
    cam1Container: document.getElementById("cam1-container"),
    cam2Container: document.getElementById("cam2-container"),
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
    tabButtons: document.querySelectorAll(".tab-button"),
    mapIframe: document.getElementById("map-iframe"),
  };

  setupTabs(ELEMENTS);
  
  // --- GANTI FUNGSI INI ---
  // setupFirebase(CONFIG, ELEMENTS); // <-- Hapus/Komentari baris ini
  setupLocalSocketIO(ELEMENTS);    // <-- Tambahkan baris ini
  // -------------------------

  setupCamera(ELEMENTS); // Ini dari supabase.js (untuk video)
  renderGallery(CONFIG, ELEMENTS); // Ini dari supabase.js (untuk galeri)
  setupModal(ELEMENTS); // Ini dari supabase.js (untuk galeri)
});

function setupTabs(elements) {
  // ... (Fungsi ini tidak berubah) ...
  elements.tabButtons.forEach((button) => {
    button.addEventListener("click", () => {
      elements.tabButtons.forEach((btn) => btn.classList.remove("active"));
      button.classList.add("active");
      const tabName = button.getAttribute("data-tab");
      elements.surfaceCaptures.classList.toggle("hidden", tabName !== "surface");
      elements.underwaterCaptures.classList.toggle(
        "hidden",
        tabName !== "underwater"
      );
    });
  });
}

// --- FUNGSI BARU UNTUK KONEKSI SOCKET.IO LOKAL ---
function setupLocalSocketIO(elements) {
  console.log("Mencoba terhubung ke server Socket.IO lokal...");
  
  // io() akan otomatis terhubung ke server yang menyajikan halaman ini
  const socket = io(); 

  socket.on("connect", () => {
    console.log("Berhasil terhubung ke server Socket.IO!");
  });

  socket.on("disconnect", () => {
    console.log("Koneksi Socket.IO terputus.");
  });

  // Ini adalah event listener utama
  socket.on("telemetry_update", (data) => {
    // 'data' adalah objek state_copy dari asv_handler.py
    // console.log(data); // Uncomment untuk debug data yang diterima

    // 1. Perbarui Info Perjalanan (Voyage Information)
    if (data.heading !== undefined) {
      elements.hdgValue.textContent = `${data.heading.toFixed(1)}°`;
    }
    if (data.speed !== undefined) {
      elements.sogValue.textContent = `${data.speed.toFixed(2)} m/s`;
    }
    // Kita gunakan nav_heading_error untuk COG, sesuai logika Firebase sebelumnya
    if (data.nav_heading_error !== undefined) {
      elements.cogValue.textContent = `${data.nav_heading_error.toFixed(1)}°`;
    }
    
    // 2. Perbarui Tanggal & Waktu
    updateDateTime(elements); // Panggil fungsi yang kita pindahkan

    // 3. Perbarui Peta & GPS
    if (data.latitude && data.longitude) {
      if (data.latitude !== lastKnownGps.lat || data.longitude !== lastKnownGps.lng) {
        lastKnownGps.lat = data.latitude;
        lastKnownGps.lng = data.longitude;

        elements.gpsValue.textContent = `${lastKnownGps.lat.toFixed(6)}, ${lastKnownGps.lng.toFixed(6)}`;
        
        // Perbarui iframe peta
        const newMapUrl = `https://maps.google.com/maps?q=${lastKnownGps.lat},${lastKnownGps.lng}&hl=id&z=18&output=embed`;
        if (elements.mapIframe) {
          elements.mapIframe.src = newMapUrl;
        }
      }
    }

    // 4. Perbarui Trajektori (Arena & Titik)
    // Logika ini meniru cloud_utils.py
    let point_to_send = 0;
    if (data.use_dummy_counter === true) {
        point_to_send = data.debug_waypoint_counter || 0;
    } else {
        const total_waypoints = (data.waypoints || []).length;
        if (total_waypoints > 0 && data.current_waypoint_index < total_waypoints) {
            point_to_send = data.current_waypoint_index + 1;
        }
    }

    if (point_to_send !== lastKnownPoint) {
        lastKnownPoint = point_to_send;
        const setPointEvent = new CustomEvent("setTrajectoryPoint", {
          detail: { point: lastKnownPoint },
        });
        window.dispatchEvent(setPointEvent);
    }

    if (data.active_arena && data.active_arena !== lastKnownArena) {
        lastKnownArena = data.active_arena;
        const switchArenaEvent = new CustomEvent("switchArena", {
          detail: { arena: lastKnownArena },
        });
        window.dispatchEvent(switchArenaEvent);
    }
  });

  socket.on("connect_error", (err) => {
    console.error("Gagal terhubung ke Socket.IO:", err);
  });
}

// --- FUNGSI YANG DIPINDAHKAN DARI FIREBASE.JS ---
function updateDateTime(elements) {
  const now = new Date();
  const days = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"];
  const options = { day: "2-digit", month: "2-digit", year: "numeric" };
  elements.dayValue.textContent = days[now.getDay()];
  elements.dateValue.textContent = now.toLocaleDateString("id-ID", options);
}