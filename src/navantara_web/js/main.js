// js/main.js

// --- KONFIGURASI IP ---
const SERVER_IP = "http://192.168.1.14:5000";

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;
let trailLine = null;
let trailCoords = [];
let waypointLayer = null;

let fullMissionWaypoints = []; 
let completedPathLayer = null; 

// Variabel global untuk state
let lastKnownArena = null;
let lastKnownPoint = -1; 
let lastKnownGps = { lat: 0, lng: 0 };

// --- [OPTIMASI: REVERSE KEY MAPPING] ---
const REVERSE_KEY_MAP = {
    "lat": "latitude",
    "lon": "longitude",
    "hdg": "heading",
    "sog": "speed",
    "bat": "battery_voltage",
    "sts": "status",
    "mode": "control_mode",
    "ar": "active_arena",
    "inv": "inverse_servo",
    "wps": "waypoints",
    "cur_wp": "current_waypoint_index",
    "wp_idx": "nav_target_wp_index",
    "wp_dst": "nav_dist_to_wp",
    "err_hdg": "nav_heading_error",
    "tgt_brg": "nav_target_bearing",
    "sat": "nav_gps_sats",
    "srv": "nav_servo_cmd",
    "mot": "nav_motor_cmd",
    "m_srv": "manual_servo_cmd",
    "m_mot": "manual_motor_cmd",
    "time": "mission_time",
    "rc": "rc_channels",
    "conn": "is_connected_to_serial",
    "dum": "use_dummy_counter",
    "dbg_cnt": "debug_waypoint_counter",
    "vis": "vision_target",
    "esp_sts": "esp_status"
};
// --- [AKHIR MAPPING] ---


document.addEventListener("DOMContentLoaded", () => {
  const ELEMENTS = {
    dayValue: document.getElementById("day-value"),
    dateValue: document.getElementById("date-value"),
    timeValue: document.getElementById("time-value"),
    gpsValue: document.getElementById("gps-value"),
    sogValue: document.getElementById("sog-value"),
    cogValue: document.getElementById("cog-value"),
    hdgValue: document.getElementById("hdg-value"),
    
    refreshGalleryBtn: document.getElementById("refresh-gallery-btn"),
    surfaceGallery: document.getElementById("surface-gallery"),        
    underwaterGallery: document.getElementById("underwater-gallery"), 
    
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
  };

  try {
    // 🚩 PENGEMBALIAN KE KOORDINAT ASLI/DEFAULT: [0.916, 104.444]
    const initialCoords = [0.916, 104.444]; 
    map = L.map("map-canvas").setView(initialCoords, 17);

    L.tileLayer("http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}", {
      maxZoom: 21,
      subdomains: ["mt0", "mt1", "mt2", "mt3"],
      attribution: "© Google Maps",
    }).addTo(map);

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
    completedPathLayer = L.layerGroup().addTo(map); 

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }

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


// === MODIFIKASI SSE: MENERAPKAN NORMALISASI KEY DAN LOGIKA MAPPING TERBARU ===
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
    if (headingLine) {
      map.removeLayer(headingLine);
      headingLine = null;
    }
  };

  eventSource.onmessage = function (event) {
    // 💡 Optimasi 1: Proses parsing JSON secepatnya
    const rawData = JSON.parse(event.data);
    if (!rawData) {
      console.warn("[SSE] Menerima data null.");
      return;
    }

    // --- [REHYDRATE KEYS] ---
    const data = {};
    Object.keys(rawData).forEach(key => {
        const longKey = REVERSE_KEY_MAP[key] || key;
        data[longKey] = rawData[key];
    });

    // 1. Normalisasi Nama Arena
    let rawArena = data.active_arena;
    let arena = null;

    if (rawArena) {
        if (rawArena.includes("B") || rawArena === "Arena_B") {
            arena = "B";
        } 
        else if (rawArena.includes("A") || rawArena === "Arena_A") {
            arena = "A";
        }
        else {
            arena = rawArena; 
        }
    }

    if (arena && arena !== lastKnownArena) {
      console.log(`[UI] Arena berubah dari ${lastKnownArena} ke ${arena} (Raw: ${rawArena})`);
      lastKnownArena = arena;
      
      const switchArenaEvent = new CustomEvent("switchArena", {
        detail: { arena: arena },
      });
      window.dispatchEvent(switchArenaEvent);
    }
    
    if (arena) {
      const mapImageDiv = document.getElementById("map-canvas");
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

    // --- Visualisasi Waypoint pada Peta Leaflet ---
    const totalWpCount = (data.waypoints && Array.isArray(data.waypoints)) ? data.waypoints.length : 0;
    
    if (data.waypoints && Array.isArray(data.waypoints)) {
      // 💡 Optimasi 2: Hanya gambar ulang Waypoint Layer jika daftar Waypoint berubah
      if (JSON.stringify(fullMissionWaypoints) !== JSON.stringify(data.waypoints)) {
        console.log(`[DIAGNOSTIC] Menerima daftar waypoint misi baru. Total: ${totalWpCount}`);
        fullMissionWaypoints = data.waypoints;
        waypointLayer.clearLayers();
        completedPathLayer.clearLayers();
      }
    }

    const targetIndex = data.nav_target_wp_index;

    // Lanjutkan proses penggambaran waypoint hanya jika ada waypoint yang dimuat
    if (targetIndex !== undefined && fullMissionWaypoints.length > 0) {
      // Clear layers dilakukan di blok di atas, ini adalah proses menggambar
      
      let completedPathCoords = [];
      waypointLayer.clearLayers(); // Hapus marker lama untuk digambar ulang

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
      
      // Hapus dan gambar ulang Completed Path Line
      completedPathLayer.clearLayers();
      if (completedPathCoords.length > 1) {
        L.polyline(completedPathCoords, {
          color: "cyan", 
          weight: 5,
        }).addTo(completedPathLayer);
      }
    }

    // 🚩 KONTROL TITIK PADA CANVAS (LOGIKA MAPPING TERBARU)
    let point = 0;
    
    if (data.use_dummy_counter === true) {
      point = data.debug_waypoint_counter || 0;
      
    } else {
      const targetWpIndex = data.nav_target_wp_index;
      
      if (targetWpIndex !== undefined && targetWpIndex >= 0) {
          
          // --- LOGIKA PEMETAAN BARU DIMULAI DI SINI ---
          switch (targetWpIndex) {
            case 0:
              point = 0; 
              break;
            case 1:
              point = 1;
              break;
            case 2:
            case 3:
              point = 2; // Waypoint 2 dan 3 memetakan ke titik ke-2
              break;
            case 4:
              point = 3; // Waypoint 4 memetakan ke titik ke-3
              break;
            case 5:
            case 6:
            case 7:
              point = 4; // Waypoint 5, 6, dan 7 memetakan ke titik ke-4
              break;
            case 8:
              point = 5; // Waypoint 8 memetakan ke titik ke-5
              break;
            case 9:
            case 10:
            case 11:
              point = 6; // Waypoint 9, 10, dan 11 memetakan ke titik ke-6
              break;
            case 12:
              point = 7; // Waypoint 12 memetakan ke titik ke-7
              break;
            case 13:
              point = 8; // Waypoint 13 memetakan ke titik ke-8
              break;
            case 14:
              point = 9; // Waypoint 14 memetakan ke titik ke-9
              break;
            default:
              // Untuk nilai di luar pemetaan, default ke nilai terakhir yang diketahui atau 0
              point = lastKnownPoint !== -1 ? lastKnownPoint : 0; 
              console.warn(`[UI Canvas] targetWpIndex ${targetWpIndex} di luar pemetaan, menggunakan point: ${point}`);
              break;
          }
          // --- LOGIKA PEMETAAN BARU BERAKHIR DI SINI ---
          
          // Batasi point agar tidak melebihi jumlah total waypoint yang dimuat dan jumlah titik di canvas (max 9)
          const maxPoints = fullMissionWaypoints.length;
          if (maxPoints > 0) {
            point = Math.min(point, maxPoints, 9); 
          }

      } else {
          // Jika Waypoint dimuat, tapi targetIndex belum diset, anggap di titik awal (0)
          point = 0; 
      }
    }
    
    // 💡 Optimasi 3: Hanya kirim event jika nilai point berubah
    if (point !== lastKnownPoint && point >= 0) { 
      console.log(`[UI Canvas] Mengirim event setTrajectoryPoint dengan point: ${point}`);
      lastKnownPoint = point;
      const setPointEvent = new CustomEvent("setTrajectoryPoint", {
        detail: { point: point },
      });
      window.dispatchEvent(setPointEvent);
    }
    
    // ----------------------------------------------------------------
    // 🚩 UPDATE DATA SENSOR (GPS & COG)
    // ----------------------------------------------------------------
    let currentLatLng = null;
    const lat = data.latitude;
    const lng = data.longitude;
    
    let isGpsDataValid = false;
    let shouldUpdateMapAndHistory = false;
    
    if (lat !== undefined && lng !== undefined) {
        try {
            if (isNaN(lat) || isNaN(lng)) {
                throw new Error("Lat/Lng bukan angka");
            }

            // 1. UPDATE TEKS GPS (DIPAKSA TAMPIL)
            if (elements.gpsValue) {
                elements.gpsValue.textContent = `${decimalToHemisphere(lat, false)} ${decimalToHemisphere(lng, true)}`;
            }
            isGpsDataValid = true;

            // 2. Cek apakah harus update peta dan riwayat (Hanya jika bergerak dan bukan (0,0))
            if (
                (lat !== 0 || lng !== 0) &&
                (lat !== lastKnownGps.lat || lng !== lastKnownGps.lng)
            ) {
                shouldUpdateMapAndHistory = true;
            }

        } catch (e) {
            console.error("Data GPS tidak valid/format salah:", e);
            if (elements.gpsValue) elements.gpsValue.textContent = "N/A";
            currentLatLng = null;
        }
    } else {
         if (elements.gpsValue) elements.gpsValue.textContent = "N/A";
    }

    // 3. UPDATE MAP DAN RIWAYAT HANYA JIKA TERJADI PERGERAKAN / PERUBAHAN
    if (shouldUpdateMapAndHistory && isGpsDataValid) {
      lastKnownGps.lat = lat;
      lastKnownGps.lng = lng;
      currentLatLng = [lat, lng];
      
      if (map && vehicleMarker) {
        vehicleMarker.setLatLng(currentLatLng);
        map.panTo(currentLatLng);
        
        // Logika Trail Line
        try {
            const last = trailCoords.length ? trailCoords[trailCoords.length - 1] : null;
            if (!last || last[0] !== currentLatLng[0] || last[1] !== currentLatLng[1]) {
              // Menambah koordinat hanya jika berbeda dari yang terakhir
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
    } else if (map && vehicleMarker && lastKnownGps.lat !== 0) {
      currentLatLng = [lastKnownGps.lat, lastKnownGps.lng];
    }
    // ----------------------------------------------------------------
    // --- END UPDATE GPS ---
    // ----------------------------------------------------------------


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
          ? `${Math.round(Math.abs(parseFloat(data.nav_heading_error)))}°`
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