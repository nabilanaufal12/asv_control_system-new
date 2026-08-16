// js/main.js

// --- KONFIGURASI IP ---
const SERVER_IP = window.location.origin; // Dinamis mendeteksi IP Jetson saat ini
let isLiveMode = true;
let currentHistoryRaceId = null;
let userIsViewingHistory = false; // True jika user manual memilih race lama

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

    surfaceGallery: document.getElementById("surface-gallery"),
    underwaterGallery: document.getElementById("underwater-gallery"),

    // --- [BARU: Elemen Log CSV] ---
    csvLogContainer: document.getElementById("csv-log-container"),
    csvLogList: document.getElementById("csv-log-list"),
    // -----------------------------

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

    const boatIcon = L.divIcon({
      html: `<svg id="boat-svg" width="30" height="30" viewBox="0 0 24 24" style="transform: rotate(0deg); transform-origin: center; transition: transform 0.2s linear;">
               <polygon points="12,2 22,22 12,17 2,22" fill="#FF0000" stroke="#FFFFFF" stroke-width="1.5"/>
             </svg>`,
      className: 'custom-boat-marker',
      iconSize: [30, 30],
      iconAnchor: [15, 15]
    });

    vehicleMarker = L.marker(initialCoords, { icon: boatIcon })
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

  // Muat daftar race secara otomatis
  fetchCsvLogList();

  if (ELEMENTS.closeModalBtn && ELEMENTS.modal) {
    ELEMENTS.closeModalBtn.addEventListener("click", () => {
      ELEMENTS.modal.style.display = "none";
    });
  }
});

let lastRenderedCaptures = "[]";

function renderGallery(daftarGambar, raceId = null) {
  const surfaceGalleryEl = document.getElementById("surface-gallery");
  const underwaterGalleryEl = document.getElementById("underwater-gallery");

  if (!surfaceGalleryEl || !underwaterGalleryEl) return;

  // Cek apakah ada perubahan daftar gambar agar tidak render ulang percuma
  const currentCapturesStr = JSON.stringify(daftarGambar);
  if (currentCapturesStr === lastRenderedCaptures) return;
  lastRenderedCaptures = currentCapturesStr;

  surfaceGalleryEl.innerHTML = "";
  underwaterGalleryEl.innerHTML = "";

  let surfaceCount = 0;
  let underwaterCount = 0;

  daftarGambar.forEach((namaFile) => {
    const img = document.createElement("img");
    // Jika raceId ada, ini history. Jika tidak, anggap live (mengambil dari /logs/ atau biarkan fetch mengarah ke endpoint live sebelumnya, tapi karena API statis kita bisa arahkan /logs/race_x/captures/ atau biarkan backend menangani via route /logs/)
    // Saat Live Mode, gambar berada di /captures/
    // Saat History, gambar berada di /logs/race_X/captures/
    let imgSrc = "";
    if (raceId) {
      imgSrc = `${SERVER_IP}/logs/race_${raceId}/captures/${namaFile}`;
    } else {
      imgSrc = `${SERVER_IP}/captures/${namaFile}`;
    }

    img.src = imgSrc;
    img.alt = namaFile;

    img.addEventListener("click", () => {
      const modal = document.getElementById("image-modal");
      const modalImg = document.getElementById("modal-img");
      const downloadBtn = document.getElementById("download-btn");
      if (modal && modalImg && downloadBtn) {
        modalImg.src = imgSrc;
        downloadBtn.href = imgSrc;
        modal.style.display = "flex";
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
}

// Fungsi manual telah dihapus karena menggunakan SSE


// --- [BARU: FUNGSI FETCH RACE LOG] ---
async function fetchCsvLogList() {
  const raceSelector = document.getElementById('race-selector');
  if (!raceSelector) return;

  try {
    const response = await fetch(`${SERVER_IP}/api/races`);
    if (!response.ok) throw new Error('Gagal mengambil data race');

    const races = await response.json();
    raceSelector.innerHTML = '<option value="">Pilih Race</option>';

      // Jika ada race, auto-load yang terbaru dan set dropdown
      if (races.length > 0) {
        races.forEach(race => {
          const option = document.createElement('option');
          option.value = race.id;
          option.textContent = race.name;
          raceSelector.appendChild(option);
        });
        const latestRace = races.reduce((prev, current) => {
          return (parseInt(prev.id) > parseInt(current.id)) ? prev : current;
        });
        raceSelector.value = latestRace.id;
        currentHistoryRaceId = String(latestRace.id);
        loadRace(latestRace.id);

        // Listener: ketika user manual pilih race, tandai sebagai history mode
        raceSelector.addEventListener('change', () => {
          const latestId = String(races.reduce((p, c) => parseInt(p.id) > parseInt(c.id) ? p : c).id);
          userIsViewingHistory = raceSelector.value !== latestId;
          console.log(`[UI] Race dipilih: ${raceSelector.value} | viewingHistory: ${userIsViewingHistory}`);
          loadRace(raceSelector.value);
        });

        console.log("Dashboard siap. Menunggu data live dari ESP32...");
      }

  } catch (error) {
    console.error("Error fetching races:", error);
  }
}

// Switch to Live Mode dihapus dari UI
function switchToLiveMode() {
  isLiveMode = true;
  currentHistoryRaceId = null;
  console.log("Switched to LIVE mode");
}

async function loadRace(raceId) {
  currentHistoryRaceId = raceId;
  console.log(`Loading gallery & logs for Race ${raceId}`);

  const dlContainer = document.getElementById('telemetry-download-container');
  if (dlContainer) {
    dlContainer.innerHTML = '<span style="color: #7f8c8d;">Memuat data telemetry...</span>';
  }

  try {
    const response = await fetch(`${SERVER_IP}/api/get-race-data?race_id=${raceId}`);
    if (response.ok) {
      const data = await response.json();

      // Plot Telemetri dan ubah tombol download
      if (data.telemetry) {
        // [FIX] HAPUS dispatch event setTrajectoryPoint menggunakan data.telemetry.length
        // karena length adalah jumlah baris CSV (ratusan), bukan index waypoint kapal.
        // Ini yang menyebabkan tiba-tiba titik trajectory langsung muncul semua.
        // window.dispatchEvent(new CustomEvent('setTrajectoryPoint', { detail: { point: data.telemetry.length } }));

        if (dlContainer) {
          const csvUrl = `${SERVER_IP}/api/races/${raceId}/download`;
          dlContainer.innerHTML = `
            <div style="display: flex; justify-content: space-between; align-items: center; width: 100%; background: rgba(10, 20, 38, 0.5); padding: 12px 18px; border: 1px solid var(--border-steel); border-radius: 8px;">
              <div style="text-align: left; line-height: 1.3;">
                <span style="color: var(--cyan); font-family: var(--font-mono); font-weight: bold; font-size: 1.1em;">mission_data.csv</span><br>
                <small style="color: var(--text-secondary); font-size: 0.85em;">Data Telemetri Race ${raceId}</small>
              </div>
              <a href="${csvUrl}" target="_blank" download class="btn-download" style="padding: 8px 16px; font-size: 0.9em;">
                <i class="fas fa-download"></i>
                <span style="margin-left: 8px;">Unduh Log</span>
              </a>
            </div>
          `;
        }
      }

      // Plot Galeri Foto
      if (data.captures) {
        lastRenderedCaptures = "[]"; // paksa render ulang
        renderGallery(data.captures, raceId);
      }
    }
  } catch (e) {
    console.error("Error load race data", e);
    if (dlContainer) {
      dlContainer.innerHTML = '<span style="color: #e74c3c;">Gagal memuat telemetry.</span>';
    }
  }
}
// ------------------------------------


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
    if (!isLiveMode) return;

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

    // --- Auto-update gallery + race selector dari SSE ---
    if (data.current_race_id != null) {
      const raceSelector = document.getElementById('race-selector');

      // Deteksi perubahan race (backend mulai race baru)
      if (String(data.current_race_id) !== String(currentHistoryRaceId)) {
        console.log(`[SSE] Race baru terdeteksi: ${currentHistoryRaceId} -> ${data.current_race_id}`);
        currentHistoryRaceId = String(data.current_race_id);

        // Tambah option race baru ke dropdown jika belum ada
        if (raceSelector) {
          const exists = Array.from(raceSelector.options).some(o => o.value === String(data.current_race_id));
          if (!exists) {
            const option = document.createElement('option');
            option.value = data.current_race_id;
            option.textContent = `Race ${data.current_race_id}`;
            raceSelector.appendChild(option);
          }

          // Jika user tidak sedang melihat history, auto-pindah ke race terbaru
          if (!userIsViewingHistory) {
            raceSelector.value = String(data.current_race_id);
            lastRenderedCaptures = "[]"; // paksa render ulang
            loadRace(data.current_race_id);
          }
        }
      }

      // Render gallery hanya jika user tidak sedang melihat history lama
      if (data.captures && Array.isArray(data.captures) && !userIsViewingHistory) {
        renderGallery(data.captures, data.current_race_id);
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

        // Hapus Titik WP, hanya rekam garis rute yang sudah/sedang dilewati
        if (i <= targetIndex) {
          completedPathCoords.push(wpLatLng);
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
        function getLogicalIndex(rawIndex) {
          if (rawIndex === 0) return 0;
          if (rawIndex >= 1 && rawIndex <= 2) return 1;
          if (rawIndex >= 3 && rawIndex <= 4) return 2;
          if (rawIndex >= 5 && rawIndex <= 6) return 3;
          if (rawIndex === 7) return 4;
          if (rawIndex >= 8 && rawIndex <= 9) return 5;
          if (rawIndex >= 10 && rawIndex <= 11) return 6;
          if (rawIndex >= 12 && rawIndex <= 15) return 7;
          if (rawIndex >= 16 && rawIndex <= 18) return 8;
          return -1;
        }

        point = getLogicalIndex(targetWpIndex);
        if (point === -1) {
          // Untuk nilai di luar pemetaan, default ke nilai terakhir yang diketahui atau 0
          point = lastKnownPoint !== -1 ? lastKnownPoint : 0;
          console.warn(`[UI Canvas] targetWpIndex ${targetWpIndex} di luar pemetaan, menggunakan point: ${point}`);
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

          // Putar marker kapal
          const boatSvg = document.getElementById("boat-svg");
          if (boatSvg) {
            boatSvg.style.transform = `rotate(${currentHdg}deg)`;
          }
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
        data.cog !== undefined
          ? `${parseFloat(data.cog).toFixed(1)}°`
          : "0.0°";
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

// --- AUTO REFRESH LOGIC ---
// Memperbarui Dropdown Race dan Capture Results secara otomatis setiap 3 detik
setInterval(async () => {
  const raceSelector = document.getElementById('race-selector');
  if (!raceSelector) return;

  // 1. Auto Refresh Dropdown Race
  try {
    const response = await fetch(`${SERVER_IP}/api/races`);
    if (response.ok) {
      const races = await response.json();

        // Update dropdown HANYA jika jumlah race berubah (ada race baru)
        if (races.length > 0 && races.length !== (raceSelector.options.length - 1)) {
          const previousValue = raceSelector.value;

          raceSelector.innerHTML = '<option value="">Pilih Race</option>';
          races.forEach(race => {
            const option = document.createElement('option');
            option.value = race.id;
            option.textContent = race.name;
            raceSelector.appendChild(option);
          });

          const latestRace = races.reduce((prev, current) => {
            return (parseInt(prev.id) > parseInt(current.id)) ? prev : current;
          });

          const oldOptionExists = races.some(r => String(r.id) === String(previousValue));

          if (!userIsViewingHistory || !oldOptionExists) {
            // Auto-pindah ke race terbaru jika user bukan sedang melihat history
            raceSelector.value = latestRace.id;
            loadRace(latestRace.id);
          } else {
            // Kembalikan ke pilihan lama jika user sedang di history
            raceSelector.value = previousValue;
          }
        }
    }
  } catch (e) { }

  // 2. Auto Refresh Gallery (Pilih race yang saat ini sedang aktif di dropdown)
  const activeValue = raceSelector.value;
  if (activeValue) {
    try {
      const response = await fetch(`${SERVER_IP}/api/get-race-data?race_id=${activeValue}`);
      if (response.ok) {
        const data = await response.json();
        if (data.captures) {
          renderGallery(data.captures, activeValue);
        }
      }
    } catch (e) { }
  }
}, 3000);

// --- Auto-Refresh / Auto-Reconnect Video Streams ---
function connectCameraStream(imgId, streamUrl) {
  const img = document.getElementById(imgId);
  if (!img) return;

  img.onload = () => {
    // Jika berhasil dimuat, biarkan
  };

  img.onerror = () => {
    // Jika stream gagal/putus, coba reconnect setelah 2 detik
    setTimeout(() => {
      img.src = `${streamUrl}?t=${new Date().getTime()}`;
    }, 2000);
  };

  // Inisialisasi awal
  img.src = `${streamUrl}?t=${new Date().getTime()}`;
}

document.addEventListener("DOMContentLoaded", function () {
  connectCameraStream("cam1-feed", `${SERVER_IP}/live_video_feed`);
  connectCameraStream("cam2-feed", `${SERVER_IP}/live_video_feed_cam2`);
});