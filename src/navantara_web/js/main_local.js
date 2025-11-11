// js/main_local.js

// === MODIFIKASI: Definisikan IP Server di satu tempat ===
const SERVER_IP = "http://192.168.1.20:5000";

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;
let trailLine = null;
let trailCoords = [];
let waypointLayer = null;

// --- [MODIFIKASI TAHAP 1: Variabel Global Baru] ---
let fullMissionWaypoints = []; // Menyimpan semua WP misi
let completedPathLayer = null; // Layer untuk garis jalur yang sudah selesai
// --- [AKHIR MODIFIKASI] ---

// Variabel global untuk state
let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 };

document.addEventListener("DOMContentLoaded", () => {
  // ... (ELEMENTS Anda tetap sama) ...
  const ELEMENTS = {
    dayValue: document.getElementById("day-value"),
    dateValue: document.getElementById("date-value"),
    timeValue: document.getElementById("time-value"),
    gpsValue: document.getElementById("gps-value"),
    sogValue: document.getElementById("sog-value"),
    cogValue: document.getElementById("cog-value"),
    hdgValue: document.getElementById("hdg-value"),
    
    // === MODIFIKASI: Elemen Galeri Baru ===
    refreshGalleryBtn: document.getElementById("refresh-gallery-btn"),
    galleryTabButtons: document.querySelectorAll(".gallery-tab-button"), // <-- BARU
    surfaceGallery: document.getElementById("surface-gallery"),       // <-- BARU
    underwaterGallery: document.getElementById("underwater-gallery"), // <-- BARU
    
    // === Elemen Modal (Tetap) ===
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
    
    // === Elemen Manual Capture Dihapus ===
  };

  // === INISIALISASI PETA LEAFLET ===
  try {
    const initialCoords = [0.916, 104.444]; // Posisi awal
    map = L.map("map-canvas").setView(initialCoords, 17);

    L.tileLayer("http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}", {
      maxZoom: 21,
      subdomains: ["mt0", "mt1", "mt2", "mt3"],
      attribution: "© Google Maps",
    }).addTo(map);

    // Marker Lingkaran Merah
    vehicleMarker = L.circleMarker(initialCoords, {
      radius: 10,
      color: "#FFFFFF",
      weight: 1.5,
      fillColor: "#FF0000",
      fillOpacity: 1,
    })
      .addTo(map)
      .bindPopup("NAVANTARA ASV");

    waypointLayer = L.layerGroup().addTo(map);
    // --- [MODIFIKASI TAHAP 2: Inisialisasi Layer Baru] ---
    completedPathLayer = L.layerGroup().addTo(map); // <-- TAMBAHKAN INI
    // --- [AKHIR MODIFIKASI] ---

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }
  // === AKHIR INISIALISASI PETA ===

  // --- [MODIFIKASI TAHAP 3: Buat Ikon Kustom] ---
  // (Pastikan path 'lib/leaflet/images/' sudah benar)
  const targetWpIcon = L.icon({
    iconUrl: "lib/leaflet/images/marker-icon.png",
    shadowUrl: "lib/leaflet/images/marker-shadow.png",
    className: "wp-target", // <-- Tambahkan kelas CSS
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
  });
  const completedWpIcon = L.icon({
    iconUrl: "lib/leaflet/images/marker-icon.png",
    shadowUrl: "lib/leaflet/images/marker-shadow.png",
    className: "wp-completed",
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
  });
  const pendingWpIcon = L.icon({
    iconUrl: "lib/leaflet/images/marker-icon.png",
    shadowUrl: "lib/leaflet/images/marker-shadow.png",
    className: "wp-pending",
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34], shadowSize: [41, 41]
  });
  // --- [AKHIR MODIFIKASI] ---

  // ... (Panggilan setupLocalSocketIO) ...
  if (typeof setupLocalSocketIO === "function") {
    // Kirim ikon yang baru dibuat ke fungsi setup
    setupLocalSocketIO(ELEMENTS, {
      targetWpIcon,
      completedWpIcon,
      pendingWpIcon,
    });
    console.log("Memulai setupLocalSocketIO (mode SSE)...");
  } else {
    console.error("Fungsi setupLocalSocketIO tidak ditemukan.");
  }
  
  // === MODIFIKASI: Panggilan refreshGallery ===
  if (ELEMENTS.refreshGalleryBtn) {
    ELEMENTS.refreshGalleryBtn.addEventListener("click", () => refreshGallery(ELEMENTS));
    refreshGallery(ELEMENTS); // Muat saat start
  }

  // === MODIFIKASI: Panggil Fungsi Setup Tab ===
  setupGalleryTabs(ELEMENTS);

  // ... (Listener Modal)
  if (ELEMENTS.closeModalBtn && ELEMENTS.modal) {
    ELEMENTS.closeModalBtn.addEventListener("click", () => {
      ELEMENTS.modal.style.display = "none";
    });
  }
  
  // === LISTENER MANUAL CAPTURE DIHAPUS ===
});

// === MODIFIKASI: Fungsi refreshGallery Diperbarui ===
/**
 * Mengambil daftar gambar dari API server dan memisahkannya
 * ke galeri Surface dan Underwater.
 */
async function refreshGallery(elements) {
  // Ambil elemen dari argumen
  const surfaceGalleryEl = elements.surfaceGallery;
  const underwaterGalleryEl = elements.underwaterGallery;
  const refreshGalleryBtn = elements.refreshGalleryBtn;

  if (!surfaceGalleryEl || !underwaterGalleryEl || !refreshGalleryBtn) {
    console.error("refreshGallery dipanggil sebelum elemen galeri siap.");
    return;
  }

  refreshGalleryBtn.textContent = "Memuat galeri...";
  refreshGalleryBtn.disabled = true;

  try {
    const response = await fetch(`${SERVER_IP}/api/gallery`);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    const daftarGambar = await response.json();

    // Kosongkan kedua galeri
    surfaceGalleryEl.innerHTML = "";
    underwaterGalleryEl.innerHTML = "";

    let surfaceCount = 0;
    let underwaterCount = 0;

    daftarGambar.forEach((namaFile) => {
      const img = document.createElement("img");
      const imgSrc = `${SERVER_IP}/captures/${namaFile}`;
      img.src = imgSrc;
      img.alt = namaFile;

      // Tambahkan listener modal (logika ini tetap sama)
      img.addEventListener("click", () => {
        const modal = document.getElementById("image-modal");
        const modalImg = document.getElementById("modal-img");
        const downloadBtn = document.getElementById("download-btn");
        if (modal && modalImg && downloadBtn) {
          modalImg.src = imgSrc;
          downloadBtn.href = imgSrc;
          modal.style.display = "block";
        }
      });

      // Logika pemisah berdasarkan nama file
      if (namaFile.startsWith("surface")) {
        surfaceGalleryEl.appendChild(img);
        surfaceCount++;
      } else if (namaFile.startsWith("underwater")) {
        underwaterGalleryEl.appendChild(img);
        underwaterCount++;
      }
    });

    if (surfaceCount === 0) {
      surfaceGalleryEl.innerHTML = "<p>Galeri Surface kosong.</p>";
    }
    if (underwaterCount === 0) {
      underwaterGalleryEl.innerHTML = "<p>Galeri Underwater kosong.</p>";
    }

  } catch (error) {
    console.error("Gagal mengambil galeri:", error);
    surfaceGalleryEl.innerHTML = "<p>Gagal memuat galeri.</p>";
    underwaterGalleryEl.innerHTML = "<p>Gagal memuat galeri.</p>";
  }

  refreshGalleryBtn.textContent = "Refresh Gallery";
  refreshGalleryBtn.disabled = false;
}

// === FUNGSI BARU: setupGalleryTabs ===
/**
 * Mengatur logika klik untuk tab galeri (Surface/Underwater).
 */
function setupGalleryTabs(elements) {
  if (elements.galleryTabButtons && elements.galleryTabButtons.length > 0) {
    elements.galleryTabButtons.forEach((button) => {
      button.addEventListener("click", () => {
        // Hapus 'active' dari semua tombol
        elements.galleryTabButtons.forEach((btn) => btn.classList.remove("active"));
        // Tambahkan 'active' ke tombol yang diklik
        button.classList.add("active");

        const tabName = button.getAttribute("data-tab");

        // Tampilkan/sembunyikan galeri yang sesuai
        if(elements.surfaceGallery) {
          elements.surfaceGallery.classList.toggle("hidden", tabName !== "surface");
        }
        if(elements.underwaterGallery) {
          elements.underwaterGallery.classList.toggle("hidden", tabName !== "underwater");
        }
      });
    });
     // Atur tab default (misal surface)
     if (elements.galleryTabButtons[0]) elements.galleryTabButtons[0].click();
  }
}


// === MODIFIKASI TOTAL: Menggunakan Server-Sent Events (SSE) ===
function setupLocalSocketIO(elements, icons) { // <-- Terima ikon kustom
  // ... (Koneksi EventSource Anda tetap utuh) ...
  const serverURL = `${SERVER_IP}/stream-telemetry`;
  console.log(`[SSE] Menghubungkan ke ${serverURL}`);
  const eventSource = new EventSource(serverURL);

  eventSource.onopen = function () {
    console.log("[SSE] Koneksi berhasil dibuka.");
    if (elements.gpsValue) elements.gpsValue.textContent = "Waiting for data...";
  };

  eventSource.onerror = function (err) {
    console.error("[SSE] Koneksi EventSource gagal:", err);
    if (elements.gpsValue) elements.gpsValue.textContent = "DISCONNECTED";
    if (headingLine) {
      map.removeLayer(headingLine);
      headingLine = null;
    }
  };

  // 5. Dipanggil SETIAP KALI data baru diterima dari server
  eventSource.onmessage = function (event) {
    const data = JSON.parse(event.data);
    if (!data) {
      console.warn("[SSE] Menerima data null.");
      return;
    }

    // --- MULAI LOGIKA UPDATE UI ---

    // Kontrol Arena & Peta Latar (Logika Anda yang ada)
    const arena = data.active_arena;
    if (arena && arena !== lastKnownArena) {
      lastKnownArena = arena;
      const switchArenaEvent = new CustomEvent("switchArena", {
        detail: { arena: arena },
      });
      window.dispatchEvent(switchArenaEvent);
    }
    if (data.active_arena) {
      const mapImageDiv = document.getElementById("map-canvas");
      if (mapImageDiv && mapImageDiv.dataset.currentArena !== data.active_arena) {
        console.log(`Mengganti peta ke Arena: ${data.active_arena}`);
        if (data.active_arena === "A") {
          mapImageDiv.style.backgroundImage = "url('images/Arena_A.png')";
        } else if (data.active_arena === "B") {
          mapImageDiv.style.backgroundImage = "url('images/Arena_B.png')";
        } else {
          mapImageDiv.style.backgroundImage = "none";
        }
        mapImageDiv.dataset.currentArena = data.active_arena;
      }
    }

    // --- [MODIFIKASI: Logika Visualisasi Waypoint Progresif] ---

    // Langkah 1: Simpan daftar waypoint lengkap jika ada di payload
    // (Ini biasanya hanya dikirim sekali saat misi di-load)
    if (data.waypoints && Array.isArray(data.waypoints)) {
      if (JSON.stringify(fullMissionWaypoints) !== JSON.stringify(data.waypoints)) {
        console.log("Menerima daftar waypoint misi baru.");
        fullMissionWaypoints = data.waypoints;
        // Saat daftar baru dimuat, bersihkan semua visualisasi lama
        waypointLayer.clearLayers();
        completedPathLayer.clearLayers();
      }
    }

    // Langkah 2: Ambil indeks target saat ini (pastikan valid)
    const targetIndex = data.nav_target_wp_index;

    // Langkah 3: Gambar ulang waypoint dan jalur berdasarkan indeks target
    if (targetIndex !== undefined && fullMissionWaypoints.length > 0) {
      // Hapus marker dan jalur lama (untuk digambar ulang)
      waypointLayer.clearLayers();
      completedPathLayer.clearLayers();

      let completedPathCoords = []; // Koordinat untuk garis Polyline

      for (let i = 0; i < fullMissionWaypoints.length; i++) {
        const wp = fullMissionWaypoints[i];
        const wpLatLng = [wp.lat, wp.lon];

        if (i < targetIndex) {
          // STATUS: SUDAH DILEWATI
          L.marker(wpLatLng, { icon: icons.completedWpIcon }) // <-- Gunakan icon
            .addTo(waypointLayer)
            .bindPopup(`WP ${i} (Completed)`);
          completedPathCoords.push(wpLatLng);

        } else if (i === targetIndex) {
          // STATUS: TARGET SAAT INI
          L.marker(wpLatLng, { icon: icons.targetWpIcon }) // <-- Gunakan icon
            .addTo(waypointLayer)
            .bindPopup(`TARGET: WP ${i}`);
          completedPathCoords.push(wpLatLng);

        } else {
          // STATUS: BELUM DIJELAJAHI
          L.marker(wpLatLng, { icon: icons.pendingWpIcon }) // <-- Gunakan icon
            .addTo(waypointLayer)
            .bindPopup(`WP ${i} (Pending)`);
        }
      }

      // Gambar garis Polyline yang menghubungkan semua titik yang sudah selesai
      if (completedPathCoords.length > 1) {
        L.polyline(completedPathCoords, {
          color: "cyan", // Warna jalur yang sudah selesai
          weight: 5,
        }).addTo(completedPathLayer);
      }
    }
    // --- [AKHIR MODIFIKASI] ---

    // Kontrol Titik (Point)
    // ... (Logika 'point' Anda tetap sama) ...
    let point = 0;
    if (data.use_dummy_counter === true) {
      point = data.debug_waypoint_counter || 0;
    } else {
      point = data.current_waypoint_index || 0; 
      if (
        data.waypoints &&
        data.waypoints.length > 0 &&
        point < data.waypoints.length
      ) {
        point = point + 1;
      } else {
        point = 0;
      }
    }
    if (point !== lastKnownPoint) {
      lastKnownPoint = point;
      const setPointEvent = new CustomEvent("setTrajectoryPoint", {
        detail: { point: point },
      });
      window.dispatchEvent(setPointEvent);
    }

    // --- (Sisa logika update UI Anda untuk GPS, SOG, COG, HDG, dll. tetap utuh) ---
    let currentLatLng = null;
    const lat = data.latitude;
    const lng = data.longitude;
    if (
      lat !== undefined &&
      lng !== undefined &&
      (lat !== lastKnownGps.lat || lng !== lastKnownGps.lng)
    ) {
      lastKnownGps.lat = lat;
      lastKnownGps.lng = lng;
      if (elements.gpsValue) {
        try {
          if (isNaN(lat) || isNaN(lng))
            throw new Error("Lat/Lng bukan angka");
          elements.gpsValue.textContent = `${decimalToHemisphere(
            lat,
            false
          )} ${decimalToHemisphere(lng, true)}`;
          currentLatLng = [lat, lng];
          if (map && vehicleMarker) {
            vehicleMarker.setLatLng(currentLatLng);
            map.panTo(currentLatLng);
            try {
              const last = trailCoords.length ? trailCoords[trailCoords.length - 1] : null;
              if (!last || last[0] !== currentLatLng[0] || last[1] !== currentLatLng[1]) {
                trailCoords.push(currentLatLng);
              }
              if (!trailLine) {
                trailLine = L.polyline(trailCoords, { color: '#00C853', weight: 3, opacity: 0.9 }).addTo(map);
              } else {
                trailLine.setLatLngs(trailCoords);
              }
            } catch (e) {
              console.warn('Error updating trail polyline', e);
            }
          }
        } catch (e) {
          console.error("Data GPS tidak valid:", e);
          elements.gpsValue.textContent = "N/A";
          currentLatLng = null;
        }
      }
    } else if (map && vehicleMarker && lastKnownGps.lat !== 0) {
      currentLatLng = [lastKnownGps.lat, lastKnownGps.lng];
    }

    let currentHdg = null;
    if (elements.hdgValue) {
      if (data.heading !== undefined) {
        try {
          const hdgNum = parseFloat(data.heading);
          if (isNaN(hdgNum)) throw new Error("HDG bukan angka");
          elements.hdgValue.textContent = `${Math.round(hdgNum)}°`;
          currentHdg = hdgNum;
        } catch (e) {
          console.error("Data HDG tidak valid:", e);
          elements.hdgValue.textContent = "N/A";
          currentHdg = null;
        }
      } else {
        elements.hdgValue.textContent = "N/A";
        currentHdg = null;
      }
    }

    if (elements.sogValue) {
      if (data.speed !== undefined) {
        try {
          const sog_ms = parseFloat(data.speed);
          if (isNaN(sog_ms)) throw new Error("SOG bukan angka");
          const sog_kmh = (sog_ms * 3.6).toFixed(1);
          const sog_knots = (sog_ms * 1.94384).toFixed(1);
          elements.sogValue.textContent = `${sog_kmh} km/jam (${sog_knots} kn)`;
        } catch (e) {
          console.error("Data SOG tidak valid:", e);
          elements.sogValue.textContent = "N/A";
        }
      } else {
        elements.sogValue.textContent = "N/A";
      }
    }

    if (elements.cogValue) {
      elements.cogValue.textContent =
        data.nav_heading_error !== undefined
          ? `${Math.round(parseFloat(data.nav_heading_error)) || 0}°`
          : "N/A";
    }

    updateDateTime(elements);

    if (map && currentLatLng && currentHdg !== null && !isNaN(currentHdg)) {
      const distanceKm = 0.05;
      const endPoint = calculateDestinationPoint(
        currentLatLng[0],
        currentLatLng[1],
        currentHdg,
        distanceKm
      );
      const linePoints = [currentLatLng, [endPoint.lat, endPoint.lng]];
      if (!headingLine) {
        headingLine = L.polyline(linePoints, {
          color: "orange",
          weight: 2,
          dashArray: "5, 10",
        }).addTo(map);
      } else {
        headingLine.setLatLngs(linePoints);
        headingLine.setStyle({ color: "orange" });
      }
    } else if (headingLine) {
      map.removeLayer(headingLine);
      headingLine = null;
    }
    // --- AKHIR LOGIKA UPDATE UI ---
  };

  // Perbarui waktu lokal setiap detik
  setInterval(() => updateDateTime(elements), 1000);
}

// === FUNGSI HELPER (TIDAK BERUBAH) ===
// ... (fungsi updateDateTime, decimalToHemisphere, calculateDestinationPoint tetap utuh) ...
function updateDateTime(elements) {
  const now = new Date();
  const days = [
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
  ];
  const dateOptions = { day: "2-digit", month: "2-digit", year: "numeric" };
  const timeOptions = {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  };

  if (elements.dayValue) {
    elements.dayValue.textContent = days[now.getDay()];
  }
  if (elements.dateValue) {
    elements.dateValue.textContent = now.toLocaleDateString("en-GB", dateOptions);
  }
  if (elements.timeValue) {
    elements.timeValue.textContent = now.toLocaleTimeString(
      "en-GB",
      timeOptions
    );
  }
}

function decimalToHemisphere(dec, isLng) {
  let absDec = Math.abs(dec);
  let hem;
  if (isLng) {
    hem = dec >= 0 ? "E" : "W";
  } else {
    hem = dec >= 0 ? "N" : "S";
  }
  return `${hem} ${absDec.toFixed(5)}`;
}

function calculateDestinationPoint(lat1, lon1, bearing, distanceKm) {
  const R = 6371; // Radius bumi dalam km
  const bearingRad = (bearing * Math.PI) / 180;
  const lat1Rad = (lat1 * Math.PI) / 180;
  const lon1Rad = (lon1 * Math.PI) / 180;
  const lat2Rad = lat1Rad + (distanceKm / R) * Math.cos(bearingRad);
  // Perhitungan Lng yang dikoreksi untuk lintang
  const lon2Rad =
    lon1Rad +
    ((distanceKm / R) * Math.sin(bearingRad)) / Math.cos(lat1Rad);
  const lat2 = (lat2Rad * 180) / Math.PI;
  const lon2 = (lon2Rad * 180) / Math.PI;
  return { lat: lat2, lng: lon2 };
}


// === FUNGSI MANUAL CAPTURE (DIHAPUS) ===