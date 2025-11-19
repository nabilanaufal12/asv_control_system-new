// js/main.js

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
  // === ELEMEN DOM DIPERBARUI ===
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
    surfaceGallery: document.getElementById("surface-gallery"),       
    underwaterGallery: document.getElementById("underwater-gallery"), 
    
    // === Elemen Modal (Tetap) ===
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
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
    completedPathLayer = L.layerGroup().addTo(map); 
    // --- [AKHIR MODIFIKASI] ---

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }
  // === AKHIR INISIALISASI PETA ===

  // --- [MODIFIKASI TAHAP 3: Buat Ikon Kustom] ---
  const targetWpIcon = L.icon({
    iconUrl: "lib/leaflet/images/marker-icon.png",
    shadowUrl: "lib/leaflet/images/marker-shadow.png",
    className: "wp-target", 
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

  if (typeof setupLocalSocketIO === "function") {
    setupLocalSocketIO(ELEMENTS, {
      targetWpIcon,
      completedWpIcon,
      pendingWpIcon,
    });
    console.log("Memulai setupLocalSocketIO (mode SSE)...");
  } else {
    console.error("Fungsi setupLocalSocketIO tidak ditemukan.");
  }
  
  if (ELEMENTS.refreshGalleryBtn) {
    ELEMENTS.refreshGalleryBtn.addEventListener("click", () => refreshGallery(ELEMENTS));
    refreshGallery(ELEMENTS); 
  }

  if (ELEMENTS.closeModalBtn && ELEMENTS.modal) {
    ELEMENTS.closeModalBtn.addEventListener("click", () => {
      ELEMENTS.modal.style.display = "none";
    });
  }
});

// === MODIFIKASI: Fungsi refreshGallery ===
async function refreshGallery(elements) {
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

    surfaceGalleryEl.innerHTML = "";
    underwaterGalleryEl.innerHTML = "";

    let surfaceCount = 0;
    let underwaterCount = 0;

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

      if (namaFile.startsWith("surface")) {
        surfaceGalleryEl.appendChild(img);
        surfaceCount++;
      } else if (namaFile.startsWith("underwater")) {
        underwaterGalleryEl.appendChild(img);
        underwaterCount++;
      }
    });

    if (surfaceCount === 0) surfaceGalleryEl.innerHTML = "<p>Galeri Surface kosong.</p>";
    if (underwaterCount === 0) underwaterGalleryEl.innerHTML = "<p>Galeri Underwater kosong.</p>";

  } catch (error) {
    console.error("Gagal mengambil galeri:", error);
    surfaceGalleryEl.innerHTML = "<p>Gagal memuat galeri.</p>";
    underwaterGalleryEl.innerHTML = "<p>Gagal memuat galeri.</p>";
  }

  refreshGalleryBtn.textContent = "Refresh Gallery";
  refreshGalleryBtn.disabled = false;
}


// === MODIFIKASI TOTAL: Menggunakan Server-Sent Events (SSE) ===
function setupLocalSocketIO(elements, icons) { 
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

  eventSource.onmessage = function (event) {
    const data = JSON.parse(event.data);
    if (!data) {
      console.warn("[SSE] Menerima data null.");
      return;
    }

    // --- [PERBAIKAN LOGIKA UPDATE UI] ---
    
    // 1. Normalisasi Nama Arena (Backend kirim "Arena_A", Frontend butuh "A")
    let rawArena = data.active_arena;
    let arena = null;

    if (rawArena) {
        // Jika mengandung huruf "B", anggap Arena B (Prioritas deteksi B agar aman)
        if (rawArena.includes("B") || rawArena === "Arena_B") {
            arena = "B";
        } 
        // Jika mengandung huruf "A", anggap Arena A
        else if (rawArena.includes("A") || rawArena === "Arena_A") {
            arena = "A";
        }
        // Fallback: gunakan nilai asli jika format lain
        else {
            arena = rawArena; 
        }
    }

    // 2. Kontrol Switch Arena (Event untuk Canvas di index.html)
    if (arena && arena !== lastKnownArena) {
      console.log(`[UI] Arena berubah dari ${lastKnownArena} ke ${arena} (Raw: ${rawArena})`);
      lastKnownArena = arena;
      
      const switchArenaEvent = new CustomEvent("switchArena", {
        detail: { arena: arena }, // Mengirim "A" atau "B" yang bersih
      });
      window.dispatchEvent(switchArenaEvent);
    }

    // 3. Kontrol Background Peta (Div #map-canvas)
    if (arena) {
      const mapImageDiv = document.getElementById("map-canvas");
      // Cek dataset.currentArena vs variable 'arena' yang sudah dinormalisasi
      if (mapImageDiv && mapImageDiv.dataset.currentArena !== arena) {
        if (arena === "A") {
          mapImageDiv.style.backgroundImage = "url('images/Arena_A.png')";
        } else if (arena === "B") {
          mapImageDiv.style.backgroundImage = "url('images/Arena_B.png')";
        } else {
          mapImageDiv.style.backgroundImage = "none";
        }
        mapImageDiv.dataset.currentArena = arena;
      }
    }
    // --- [AKHIR PERBAIKAN] ---

    // --- [MODIFIKASI: Logika Visualisasi Waypoint Progresif] ---
    if (data.waypoints && Array.isArray(data.waypoints)) {
      if (JSON.stringify(fullMissionWaypoints) !== JSON.stringify(data.waypoints)) {
        console.log("Menerima daftar waypoint misi baru.");
        fullMissionWaypoints = data.waypoints;
        waypointLayer.clearLayers();
        completedPathLayer.clearLayers();
      }
    }

    const targetIndex = data.nav_target_wp_index;

    if (targetIndex !== undefined && fullMissionWaypoints.length > 0) {
      waypointLayer.clearLayers();
      completedPathLayer.clearLayers();

      let completedPathCoords = [];

      for (let i = 0; i < fullMissionWaypoints.length; i++) {
        const wp = fullMissionWaypoints[i];
        const wpLatLng = [wp.lat, wp.lon];

        if (i < targetIndex) {
          L.marker(wpLatLng, { icon: icons.completedWpIcon })
            .addTo(waypointLayer)
            .bindPopup(`WP ${i} (Completed)`);
          completedPathCoords.push(wpLatLng);

        } else if (i === targetIndex) {
          L.marker(wpLatLng, { icon: icons.targetWpIcon })
            .addTo(waypointLayer)
            .bindPopup(`TARGET: WP ${i}`);
          completedPathCoords.push(wpLatLng);

        } else {
          L.marker(wpLatLng, { icon: icons.pendingWpIcon })
            .addTo(waypointLayer)
            .bindPopup(`WP ${i} (Pending)`);
        }
      }

      if (completedPathCoords.length > 1) {
        L.polyline(completedPathCoords, {
          color: "cyan", 
          weight: 5,
        }).addTo(completedPathLayer);
      }
    }
    // --- [AKHIR MODIFIKASI] ---

    // Kontrol Titik (Point)
    let point = 0;
    if (data.use_dummy_counter === true) {
      point = data.debug_waypoint_counter || 0;
    } else {
      point = data.nav_target_wp_index || 0; 
      
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

    // --- Update Data Sensor ---
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
  };

  setInterval(() => updateDateTime(elements), 1000);
}

// === FUNGSI HELPER ===
function updateDateTime(elements) {
  const now = new Date();
  const days = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
  const dateOptions = { day: "2-digit", month: "2-digit", year: "numeric" };
  const timeOptions = { hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false };

  if (elements.dayValue) elements.dayValue.textContent = days[now.getDay()];
  if (elements.dateValue) elements.dateValue.textContent = now.toLocaleDateString("en-GB", dateOptions);
  if (elements.timeValue) elements.timeValue.textContent = now.toLocaleTimeString("en-GB", timeOptions);
}

function decimalToHemisphere(dec, isLng) {
  let absDec = Math.abs(dec);
  let hem = isLng ? (dec >= 0 ? "E" : "W") : (dec >= 0 ? "N" : "S");
  return `${hem} ${absDec.toFixed(5)}`;
}

function calculateDestinationPoint(lat1, lon1, bearing, distanceKm) {
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