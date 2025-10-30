// js/main_local.js

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;

// Variabel global untuk state
let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 };


document.addEventListener("DOMContentLoaded", () => {
  // MODIFIKASI: Hapus config online
  // const CONFIG = { ... };

  const ELEMENTS = {
    // Urutan sesuai HTML baru
    dayValue: document.getElementById("day-value"),
    dateValue: document.getElementById("date-value"),
    timeValue: document.getElementById("time-value"),
    gpsValue: document.getElementById("gps-value"),
    sogValue: document.getElementById("sog-value"),
    cogValue: document.getElementById("cog-value"),
    hdgValue: document.getElementById("hdg-value"),

    // Elemen lainnya
    surfaceCaptures: document.getElementById("surface-captures"),
    underwaterCaptures: document.getElementById("underwater-captures"),
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
    tabButtons: document.querySelectorAll(".tab-button"),
    refreshGalleryBtn: document.getElementById("refresh-gallery-btn"), // Tombol refresh
  };

  // === INISIALISASI PETA LEAFLET ===
  try {
    const initialCoords = [0.916, 104.444]; // Posisi awal
    map = L.map('map-canvas').setView(initialCoords, 17);

    L.tileLayer('http://{s}.google.com/vt/lyrs=s,h&x={x}&y={y}&z={z}',{
        maxZoom: 21,
        subdomains:['mt0','mt1','mt2','mt3'],
        attribution: '© Google Maps'
    }).addTo(map);

    // Marker Lingkaran Merah
    vehicleMarker = L.circleMarker(initialCoords, {
        radius: 6, color: '#FFFFFF', weight: 1.5, fillColor: '#FF0000', fillOpacity: 1
      }).addTo(map)
      .bindPopup('NAVANTARA ASV');

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }
  // === AKHIR INISIALISASI PETA ===

  // Panggilan fungsi Anda yang lain
  setupTabs(ELEMENTS);

  // === MODIFIKASI: Panggil setupLocalSocketIO (yang sekarang isinya SSE) ===
  if (typeof setupLocalSocketIO === 'function') {
    // Tidak perlu CONFIG, hanya ELEMENTS
    setupLocalSocketIO(ELEMENTS); 
    console.log("Memulai setupLocalSocketIO (mode SSE)...");
  } else {
    console.error("Fungsi setupLocalSocketIO tidak ditemukan.");
  }
  // === AKHIR MODIFIKASI ===

  // Sembunyikan tombol refresh galeri
  if (ELEMENTS.refreshGalleryBtn) {
      ELEMENTS.refreshGalleryBtn.style.display = 'none';
  }
});

// Fungsi setupTabs Anda (Tidak berubah)
function setupTabs(elements) {
  if (elements.tabButtons && elements.tabButtons.length > 0) {
    elements.tabButtons.forEach((button) => {
      button.addEventListener("click", () => {
        elements.tabButtons.forEach((btn) => btn.classList.remove("active"));
        button.classList.add("active");
        const tabName = button.getAttribute("data-tab");
        if(elements.surfaceCaptures) {
          elements.surfaceCaptures.classList.toggle("hidden", tabName !== "surface");
        }
        if(elements.underwaterCaptures) {
          elements.underwaterCaptures.classList.toggle(
            "hidden",
            tabName !== "underwater"
          );
        }
      });
    });
     // Atur tab default (misal surface)
     if (elements.tabButtons[0]) elements.tabButtons[0].click();
  }
}


// === MODIFIKASI TOTAL: Menggunakan Server-Sent Events (SSE) ===

/**
 * Memulai koneksi Server-Sent Events (SSE) lokal dan mengatur listener.
 * (Menggantikan fungsi Socket.IO yang lama)
 */
function setupLocalSocketIO(elements) {
    
    // 1. Tentukan URL endpoint streaming baru kita.
    // IP 192.168.1.20 diambil dari config.json Anda.
    const serverURL = 'http://192.168.1.18:5000/stream-telemetry'; 

    console.log(`[SSE] Menghubungkan ke ${serverURL}`);

    // 2. Buat koneksi EventSource
    const eventSource = new EventSource(serverURL);

    // 3. Dipanggil saat koneksi berhasil dibuka
    eventSource.onopen = function() {
        console.log('[SSE] Koneksi berhasil dibuka.');
        if (elements.gpsValue) elements.gpsValue.textContent = "Waiting for data...";
    };

    // 4. Dipanggil jika terjadi error koneksi
    eventSource.onerror = function(err) {
        console.error("[SSE] Koneksi EventSource gagal:", err);
        if (elements.gpsValue) elements.gpsValue.textContent = "DISCONNECTED";
        if (headingLine) {
            map.removeLayer(headingLine);
            headingLine = null;
        }
    };

    // 5. Dipanggil SETIAP KALI data baru diterima dari server
    eventSource.onmessage = function(event) {
        
        // 'event.data' adalah string JSON yang dikirim dari Flask
        const data = JSON.parse(event.data); 

        if (!data) {
            console.warn("[SSE] Menerima data null.");
            return;
        }

        // --- MULAI LOGIKA UPDATE UI (Logika ini SAMA PERSIS dengan kode Anda sebelumnya) ---
        
        // Kontrol Arena
        const arena = data.active_arena; // <-- Key SUDAH BENAR
        if (arena && arena !== lastKnownArena) {
            lastKnownArena = arena;
            const switchArenaEvent = new CustomEvent("switchArena", { detail: { arena: arena } });
            window.dispatchEvent(switchArenaEvent);
        }

        // Kontrol Titik (Point)
        let point = 0;
        if (data.use_dummy_counter === true) { // <-- Key SUDAH BENAR
            point = data.debug_waypoint_counter || 0; // <-- Key SUDAH BENAR
        } else {
            point = (data.current_waypoint_index || 0); // <-- Key SUDAH BENAR (index berbasis 0)
            
            if (data.waypoints && data.waypoints.length > 0 && point < data.waypoints.length) {
                 point = point + 1; // Konversi ke berbasis 1 untuk tampilan
            } else {
                 point = 0; // Default
            }
        }

        if (point !== lastKnownPoint) {
            lastKnownPoint = point;
            const setPointEvent = new CustomEvent("setTrajectoryPoint", { detail: { point: point } });
            window.dispatchEvent(setPointEvent);
        }

        // --- PEMBARUAN PETA GPS & GARIS HDG ---
        let currentLatLng = null;
        
        // === BACA data.latitude dan data.longitude ===
        const lat = data.latitude;
        const lng = data.longitude;
        
        if (lat !== undefined && lng !== undefined && (lat !== lastKnownGps.lat || lng !== lastKnownGps.lng)) {
            lastKnownGps.lat = lat;
            lastKnownGps.lng = lng;

            if (elements.gpsValue) {
                try {
                    if (isNaN(lat) || isNaN(lng)) throw new Error("Lat/Lng bukan angka");
                    elements.gpsValue.textContent = `${decimalToHemisphere(lat, false)} ${decimalToHemisphere(lng, true)}`;
                    currentLatLng = [lat, lng];
                    if (map && vehicleMarker) {
                        vehicleMarker.setLatLng(currentLatLng);
                        map.panTo(currentLatLng);
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
        // --- AKHIR BAGIAN POSISI GPS ---

        // --- PEMBARUAN ELEMEN HTML LAINNYA ---
        let currentHdg = null;
        if (elements.hdgValue) {
            // === BACA data.heading ===
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

        // === BACA data.speed dan KONVERSI ke km/jam ===
        if (elements.sogValue) {
            if (data.speed !== undefined) { // Baca 'speed' (m/s)
                try {
                    const sog_ms = parseFloat(data.speed); // Ini m/s
                    if (isNaN(sog_ms)) throw new Error("SOG bukan angka");

                    const sog_kmh = (sog_ms * 3.6).toFixed(1); // Konversi m/s ke km/jam
                    const sog_knots = (sog_ms * 1.94384).toFixed(1); // Konversi m/s ke knots
                    
                    elements.sogValue.textContent = `${sog_kmh} km/jam (${sog_knots} kn)`;
                } catch (e) {
                    console.error("Data SOG tidak valid:", e);
                    elements.sogValue.textContent = "N/A";
                }
            } else {
                elements.sogValue.textContent = "N/A";
            }
        }
        
        // Update COG (Kita gunakan nav_heading_error sebagai COG di monitor ini)
        if (elements.cogValue) {
            // === BACA data.nav_heading_error ===
            elements.cogValue.textContent =
            data.nav_heading_error !== undefined ? `${Math.round(parseFloat(data.nav_heading_error)) || 0}°` : "N/A";
        }

        // Update HARI, TANGGAL, dan WAKTU (menggunakan waktu LOKAL BROWSER)
        updateDateTime(elements);

        // --- MEMBUAT/MEMPERBARUI GARIS ARAH HDG (ORANGE) ---
        if (map && currentLatLng && currentHdg !== null && !isNaN(currentHdg)) {
            const distanceKm = 0.05; // 50 meter
            const endPoint = calculateDestinationPoint(currentLatLng[0], currentLatLng[1], currentHdg, distanceKm);
            const linePoints = [ currentLatLng, [endPoint.lat, endPoint.lng] ];

            if (!headingLine) {
                headingLine = L.polyline(linePoints, {
                    color: 'orange',      // Warna garis jadi oranye
                    weight: 2,
                    dashArray: '5, 10'
                }).addTo(map);
            } else {
                headingLine.setLatLngs(linePoints);
                headingLine.setStyle({ color: 'orange' }); // Pastikan warna tetap
            }
        } else if (headingLine) {
            // Hapus garis jika GPS atau HDG tidak valid
            map.removeLayer(headingLine);
            headingLine = null;
        }
        // --- AKHIR LOGIKA UPDATE UI ---
    }

    // Perbarui waktu lokal setiap detik
    setInterval(() => updateDateTime(elements), 1000);
}


// === FUNGSI HELPER (TIDAK BERUBAH) ===

/**
 * Memperbarui elemen DAY, DATE, dan TIME menggunakan waktu lokal browser.
 */
function updateDateTime(elements) {
  const now = new Date();
  const days = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
  const dateOptions = { day: "2-digit", month: "2-digit", year: "numeric" };
  const timeOptions = { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false };
  
  if (elements.dayValue) {
    elements.dayValue.textContent = days[now.getDay()];
  }
  if (elements.dateValue) {
    elements.dateValue.textContent = now.toLocaleDateString("en-GB", dateOptions);
  }
  if (elements.timeValue) {
      elements.timeValue.textContent = now.toLocaleTimeString("en-GB", timeOptions);
  }
}

/**
 * Konversi GPS desimal ke format N/S/E/W.
 */
function decimalToHemisphere(dec, isLng) {
    let absDec = Math.abs(dec);
    let hem;
    if (isLng) { hem = dec >= 0 ? 'E' : 'W'; }
    else { hem = dec >= 0 ? 'N' : 'S'; }
    return `${hem} ${absDec.toFixed(5)}`;
}

/**
 * Menghitung titik akhir GPS berdasarkan titik awal, arah, dan jarak.
 */
function calculateDestinationPoint(lat1, lon1, bearing, distanceKm) {
    const R = 6371; // Radius bumi dalam km
    const bearingRad = bearing * Math.PI / 180;
    const lat1Rad = lat1 * Math.PI / 180;
    const lon1Rad = lon1 * Math.PI / 180;
    const lat2Rad = lat1Rad + (distanceKm / R) * Math.cos(bearingRad);
    // Perhitungan Lng yang dikoreksi untuk lintang
    const lon2Rad = lon1Rad + (distanceKm / R) * Math.sin(bearingRad) / Math.cos(lat1Rad);
    const lat2 = lat2Rad * 180 / Math.PI;
    const lon2 = lon2Rad * 180 / Math.PI;
    return { lat: lat2, lng: lon2 };
}