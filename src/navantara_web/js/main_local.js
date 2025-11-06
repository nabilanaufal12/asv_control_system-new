// js/main_local.js

// === MODIFIKASI: Definisikan IP Server di satu tempat ===
// IP 192.168.1.18 diambil dari config.json Anda.
const SERVER_IP = "http://192.168.1.9:5000";

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;
let trailLine = null;
let trailCoords = [];
let waypointLayer = null; // <-- [MODIFIKASI] Tambahkan layer group untuk waypoints

// Variabel global untuk state
let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 };

document.addEventListener("DOMContentLoaded", () => {
  // MODIFIKASI: Hapus config online
  // const CONFIG = { ... };

  const ELEMENTS = {
    // ... (Elemen Anda tetap sama) ...
    dayValue: document.getElementById("day-value"),
    dateValue: document.getElementById("date-value"),
    timeValue: document.getElementById("time-value"),
    gpsValue: document.getElementById("gps-value"),
    sogValue: document.getElementById("sog-value"),
    cogValue: document.getElementById("cog-value"),
    hdgValue: document.getElementById("hdg-value"),
    galleryImages: document.getElementById("gallery-images"),
    refreshGalleryBtn: document.getElementById("refresh-gallery-btn"),
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
    manualCaptureSurface: document.getElementById("manual-capture-surface"),
    manualCaptureUnderwater: document.getElementById("manual-capture-underwater"),
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
      radius: 6,
      color: "#FFFFFF",
      weight: 1.5,
      fillColor: "#FF0000",
      fillOpacity: 1,
    })
      .addTo(map)
      .bindPopup("NAVANTARA ASV");
    
    // <-- [MODIFIKASI] Inisialisasi waypoint layer
    waypointLayer = L.layerGroup().addTo(map);

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }
  // === AKHIR INISIALISASI PETA ===

  // ... (setupLocalSocketIO, listener galeri, listener modal, listener capture manual tetap sama) ...
  if (typeof setupLocalSocketIO === "function") {
    setupLocalSocketIO(ELEMENTS);
    console.log("Memulai setupLocalSocketIO (mode SSE)...");
  } else {
    console.error("Fungsi setupLocalSocketIO tidak ditemukan.");
  }
  
  if (ELEMENTS.refreshGalleryBtn && ELEMENTS.galleryImages) {
    ELEMENTS.refreshGalleryBtn.addEventListener("click", refreshGallery);
    refreshGallery();
  } else {
    console.warn("Elemen galeri (tombol/div) tidak ditemukan.");
  }
  
  if (ELEMENTS.closeModalBtn && ELEMENTS.modal) {
    ELEMENTS.closeModalBtn.addEventListener("click", () => {
      ELEMENTS.modal.style.display = "none";
    });
  }
  
  if (ELEMENTS.manualCaptureSurface) {
    ELEMENTS.manualCaptureSurface.addEventListener("click", async () => {
      await handleManualCapture("surface");
    });
  }
  if (ELEMENTS.manualCaptureUnderwater) {
    ELEMENTS.manualCaptureUnderwater.addEventListener("click", async () => {
      await handleManualCapture("underwater");
    });
  }
});

// ... (Fungsi refreshGallery tetap sama) ...
async function refreshGallery() {
  // ... (implementasi refreshGallery Anda) ...
  const galleryImagesEl = document.getElementById("gallery-images");
  const refreshGalleryBtn = document.getElementById("refresh-gallery-btn");

  if (!galleryImagesEl || !refreshGalleryBtn) {
    console.error("refreshGallery dipanggil sebelum elemen siap.");
    return;
  }
  refreshGalleryBtn.textContent = "Memuat galeri...";
  refreshGalleryBtn.disabled = true;
  try {
    const response = await fetch(`${SERVER_IP}/api/gallery`);
    if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
    const daftarGambar = await response.json();
    galleryImagesEl.innerHTML = "";
    if (daftarGambar.length === 0) {
      galleryImagesEl.innerHTML = "<p>Galeri masih kosong.</p>";
    }
    daftarGambar.forEach((namaFile) => {
      const img = document.createElement("img");
      const imgSrc = `${SERVER_IP}/captures/${namaFile}`;
      img.src = imgSrc;
      img.alt = namaFile;
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
      galleryImagesEl.appendChild(img);
    });
  } catch (error) {
    console.error("Gagal mengambil galeri:", error);
    galleryImagesEl.innerHTML = "<p>Gagal memuat galeri. Periksa konsol.</p>";
  }
  refreshGalleryBtn.textContent = "Refresh Gallery";
  refreshGalleryBtn.disabled = false;
}

// === MODIFIKASI TOTAL: Menggunakan Server-Sent Events (SSE) ===
function setupLocalSocketIO(elements) {
  // ... (URL, EventSource, onopen, onerror tetap sama) ...
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
    // 'event.data' adalah string JSON yang dikirim dari Flask
    const data = JSON.parse(event.data);

    if (!data) {
      console.warn("[SSE] Menerima data null.");
      return;
    }

    // --- MULAI LOGIKA UPDATE UI ---

    // Kontrol Arena (Logika event custom Anda yang ada)
    const arena = data.active_arena; 
    if (arena && arena !== lastKnownArena) {
      lastKnownArena = arena;
      const switchArenaEvent = new CustomEvent("switchArena", {
        detail: { arena: arena },
      });
      window.dispatchEvent(switchArenaEvent);
    }
    
    // --- [MODIFIKASI BARU: GANTI GAMBAR PETA BERDASARKAN ARENA] ---
    if (data.active_arena) {
        const mapImageDiv = document.getElementById('map-canvas'); // Target div peta
        if (mapImageDiv && mapImageDiv.dataset.currentArena !== data.active_arena) {
            console.log(`Mengganti peta ke Arena: ${data.active_arena}`);
            if (data.active_arena === 'A') {
                // Path relatif dari folder js ke folder images (jika struktur folder Anda /js, /images)
                // Jika monitor_local.html ada di root, pathnya adalah 'images/Arena_A.png'
                // Asumsi monitor_local.html ada di root:
                mapImageDiv.style.backgroundImage = "url('images/Arena_A.png')";
            } else if (data.active_arena === 'B') {
                mapImageDiv.style.backgroundImage = "url('images/Arena_B.png')";
            } else {
                mapImageDiv.style.backgroundImage = "none"; // Hapus gambar
            }
            mapImageDiv.dataset.currentArena = data.active_arena; // Simpan status
        }
    }
    // --- [AKHIR MODIFIKASI BARU] ---


    // --- [MODIFIKASI BARU: LOGIKA WAYPOINT] ---
    // Cek jika payload waypoints ada (bahkan jika kosong)
    if (data.waypoints && Array.isArray(data.waypoints) && waypointLayer) {
        waypointLayer.clearLayers(); // Hapus waypoint lama
        
        // Jika listnya kosong (dari missions.py baru), tidak ada yang akan ditambahkan.
        // Jika nanti Anda mengirim waypoint (Tahap 3), ini akan otomatis menggambarnya.
        data.waypoints.forEach(function (wp) {
            L.marker([wp.lat, wp.lon]) // (Icon default)
                .addTo(waypointLayer)
                .bindPopup(`WP: ${wp.lat}, ${wp.lon}`);
        });
    }
    // --- [AKHIR MODIFIKASI BARU] ---


    // Kontrol Titik (Point) - (Logika Anda yang ada tetap di sini)
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

    // --- PEMBARUAN PETA GPS & GARIS HDG ---
    // ... (Logika GPS, HDG, SOG, COG, DateTime, dan Heading Line Anda tetap sama) ...
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
          if (isNaN(lat) || isNaN(lng)) throw new Error("Lat/Lng bukan angka");
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
function updateDateTime(elements) {
  // ... (implementasi Anda) ...
  const now = new Date();
  const days = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
  const dateOptions = { day: "2-digit", month: "2-digit", year: "numeric" };
  const timeOptions = { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false };
  if (elements.dayValue) elements.dayValue.textContent = days[now.getDay()];
  if (elements.dateValue) elements.dateValue.textContent = now.toLocaleDateString("en-GB", dateOptions);
  if (elements.timeValue) elements.timeValue.textContent = now.toLocaleTimeString("en-GB", timeOptions);
}

function decimalToHemisphere(dec, isLng) {
  // ... (implementasi Anda) ...
  let absDec = Math.abs(dec);
  let hem;
  if (isLng) hem = dec >= 0 ? "E" : "W";
  else hem = dec >= 0 ? "N" : "S";
  return `${hem} ${absDec.toFixed(5)}`;
}

function calculateDestinationPoint(lat1, lon1, bearing, distanceKm) {
  // ... (implementasi Anda) ...
  const R = 6371;
  const bearingRad = (bearing * Math.PI) / 180;
  const lat1Rad = (lat1 * Math.PI) / 180;
  const lon1Rad = (lon1 * Math.PI) / 180;
  const lat2Rad = lat1Rad + (distanceKm / R) * Math.cos(bearingRad);
  const lon2Rad = lon1Rad + ((distanceKm / R) * Math.sin(bearingRad)) / Math.cos(lat1Rad);
  const lat2 = (lat2Rad * 180) / Math.PI;
  const lon2 = (lon2Rad * 180) / Math.PI;
  return { lat: lat2, lng: lon2 };
}

// === FUNGSI BARU: Handle Manual Capture (Tetap sama) ===
async function handleManualCapture(camType) {
  // ... (implementasi Anda) ...
  let imgEl = null;
  let btn = null;
  if (camType === "surface") {
    imgEl = document.querySelector('#cam1-container img');
    btn = document.getElementById("manual-capture-surface");
  } else if (camType === "underwater") {
    imgEl = document.querySelector('#cam2-container img');
    btn = document.getElementById("manual-capture-underwater");
  } else {
    alert("Jenis kamera tidak valid!"); return;
  }
  if (!imgEl) {
    alert("Stream kamera tidak ditemukan!"); return;
  }
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Memotret...";
  }
  try {
    const canvas = document.createElement('canvas');
    canvas.width = imgEl.naturalWidth || imgEl.width;
    canvas.height = imgEl.naturalHeight || imgEl.height;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(imgEl, 0, 0, canvas.width, canvas.height);
    const dataURL = canvas.toDataURL('image/jpeg');
    addImageToGallery(dataURL, camType);
  } catch (err) {
    alert("Gagal mengambil foto: " + err.message);
  }
  if (btn) {
    btn.disabled = false;
    btn.textContent = camType === "surface" ? "Potret Atas (S)" : "Pottret Bawah (U)";
  }
}

function addImageToGallery(dataURL, camType) {
  // ... (implementasi Anda) ...
  const galleryImagesEl = document.getElementById("gallery-images");
  if (!galleryImagesEl) return;
  const img = document.createElement("img");
  img.src = dataURL;
  img.alt = camType === "surface" ? "Surface Capture" : "Underwater Capture";
  img.title = camType === "surface" ? "Surface Capture" : "Underwater Capture";
  img.addEventListener("click", () => {
    const modal = document.getElementById("image-modal");
    const modalImg = document.getElementById("modal-img");
    const downloadBtn = document.getElementById("download-btn");
    if (modal && modalImg && downloadBtn) {
      modalImg.src = dataURL;
      downloadBtn.href = dataURL;
      modal.style.display = "block";
    }
  });
  galleryImagesEl.prepend(img);
}