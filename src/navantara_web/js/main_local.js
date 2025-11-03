// js/main_local.js

// === MODIFIKASI: Definisikan IP Server di satu tempat ===
// IP 192.168.1.18 diambil dari config.json Anda.
const SERVER_IP = "http://192.168.1.20:5000";

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;
let trailLine = null;
let trailCoords = [];

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

    // --- MODIFIKASI: Hapus elemen tab, tambahkan galeri ---
    galleryImages: document.getElementById("gallery-images"), // <-- BARU
    refreshGalleryBtn: document.getElementById("refresh-gallery-btn"), // Tombol refresh
    // surfaceCaptures: ... (DIHAPUS)
    // underwaterCaptures: ... (DIHAPUS)
    // tabButtons: ... (DIHAPUS)
    // --- AKHIR MODIFIKASI ---

    // Elemen Modal (Tetap)
    modal: document.getElementById("image-modal"),
    modalImg: document.getElementById("modal-img"),
    downloadBtn: document.getElementById("download-btn"),
    closeModalBtn: document.getElementById("close-modal"),
    // Tombol manual capture
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

    console.log("Peta Leaflet (Google Satellite) berhasil dimuat.");
  } catch (e) {
    console.error("Gagal memuat Peta Leaflet.", e);
  }
  // === AKHIR INISIALISASI PETA ===

  // --- MODIFIKASI: Hapus pemanggilan setupTabs ---
  // setupTabs(ELEMENTS); (DIHAPUS)
  // --- AKHIR MODIFIKASI ---

  // === MODIFIKASI: Panggil setupLocalSocketIO (yang sekarang isinya SSE) ===
  if (typeof setupLocalSocketIO === "function") {
    // Tidak perlu CONFIG, hanya ELEMENTS
    setupLocalSocketIO(ELEMENTS);
    console.log("Memulai setupLocalSocketIO (mode SSE)...");
  } else {
    console.error("Fungsi setupLocalSocketIO tidak ditemukan.");
  }
  // === AKHIR MODIFIKASI ===

  // --- MODIFIKASI: Ganti logika 'display:none' dengan event listener ---
  if (ELEMENTS.refreshGalleryBtn && ELEMENTS.galleryImages) {
    // Pasang listener ke tombol
    ELEMENTS.refreshGalleryBtn.addEventListener("click", refreshGallery);

    // (Opsional) Langsung muat galeri saat halaman pertama kali dibuka
    refreshGallery();
  } else {
    console.warn("Elemen galeri (tombol/div) tidak ditemukan.");
  }
  // --- AKHIR MODIFIKASI ---

  // --- MODIFIKASI: Tambahkan listener untuk tombol tutup modal ---
  if (ELEMENTS.closeModalBtn && ELEMENTS.modal) {
    ELEMENTS.closeModalBtn.addEventListener("click", () => {
      ELEMENTS.modal.style.display = "none";
    });
  }
  // --- AKHIR MODIFIKASI ---

  // === MODIFIKASI BARU: Tombol Potret Atas & Bawah ===
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
  // === AKHIR MODIFIKASI BARU ===
});

// --- MODIFIKASI: Hapus fungsi setupTabs ---
// function setupTabs(elements) { ... } (SELURUH FUNGSI DIHAPUS)
// --- AKHIR MODIFIKASI ---

// === FUNGSI BARU: Implementasi Galeri dari Prototipe ===
/**
 * Mengambil daftar gambar dari API server dan menampilkannya di galeri.
 * (Berdasarkan prototipe coba.html)
 */
async function refreshGallery() {
  // Ambil elemen dari scope global (didefinisikan di DOMContentLoaded)
  const galleryImagesEl = document.getElementById("gallery-images");
  const refreshGalleryBtn = document.getElementById("refresh-gallery-btn");

  if (!galleryImagesEl || !refreshGalleryBtn) {
    console.error("refreshGallery dipanggil sebelum elemen siap.");
    return;
  }

  refreshGalleryBtn.textContent = "Memuat galeri...";
  refreshGalleryBtn.disabled = true; // Nonaktifkan tombol saat loading

  try {
    // 1. Meminta (fetch) daftar gambar dari server kita
    const response = await fetch(`${SERVER_IP}/api/gallery`);

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const daftarGambar = await response.json(); // Dapat array JSON [ "img1.jpg", "img2.jpg" ]

    galleryImagesEl.innerHTML = ""; // Kosongkan galeri

    if (daftarGambar.length === 0) {
      galleryImagesEl.innerHTML = "<p>Galeri masih kosong.</p>";
    }

    // 2. Looping dan buat tag <img> untuk setiap nama file
    daftarGambar.forEach((namaFile) => {
      const img = document.createElement("img");
      const imgSrc = `${SERVER_IP}/captures/${namaFile}`; // -> http://.../captures/namafile.jpg

      img.src = imgSrc;
      img.alt = namaFile;

      // 3. (Bonus) Tambahkan listener untuk membuka modal
      img.addEventListener("click", () => {
        const modal = document.getElementById("image-modal");
        const modalImg = document.getElementById("modal-img");
        const downloadBtn = document.getElementById("download-btn");

        if (modal && modalImg && downloadBtn) {
          modalImg.src = imgSrc;
          downloadBtn.href = imgSrc; // Set link download
          modal.style.display = "block"; // Tampilkan modal
        }
      });

      galleryImagesEl.appendChild(img);
    });
  } catch (error) {
    console.error("Gagal mengambil galeri:", error);
    galleryImagesEl.innerHTML =
      "<p>Gagal memuat galeri. Periksa konsol.</p>";
  }

  refreshGalleryBtn.textContent = "Refresh Gallery";
  refreshGalleryBtn.disabled = false; // Aktifkan kembali tombol
}
// === AKHIR FUNGSI BARU ===

// === MODIFIKASI TOTAL: Menggunakan Server-Sent Events (SSE) ===

/**
 * Memulai koneksi Server-Sent Events (SSE) lokal dan mengatur listener.
 * (Menggantikan fungsi Socket.IO yang lama)
 */
function setupLocalSocketIO(elements) {
  // 1. Tentukan URL endpoint streaming baru kita.
  const serverURL = `${SERVER_IP}/stream-telemetry`; // Menggunakan konstanta global

  console.log(`[SSE] Menghubungkan ke ${serverURL}`);

  // 2. Buat koneksi EventSource
  const eventSource = new EventSource(serverURL);

  // 3. Dipanggil saat koneksi berhasil dibuka
  eventSource.onopen = function () {
    console.log("[SSE] Koneksi berhasil dibuka.");
    if (elements.gpsValue) elements.gpsValue.textContent = "Waiting for data...";
  };

  // 4. Dipanggil jika terjadi error koneksi
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

    // --- MULAI LOGIKA UPDATE UI (Logika ini SAMA PERSIS dengan kode Anda sebelumnya) ---

    // Kontrol Arena
    const arena = data.active_arena; // <-- Key SUDAH BENAR
    if (arena && arena !== lastKnownArena) {
      lastKnownArena = arena;
      const switchArenaEvent = new CustomEvent("switchArena", {
        detail: { arena: arena },
      });
      window.dispatchEvent(switchArenaEvent);
    }

    // Kontrol Titik (Point)
    let point = 0;
    if (data.use_dummy_counter === true) {
      // <-- Key SUDAH BENAR
      point = data.debug_waypoint_counter || 0; // <-- Key SUDAH BENAR
    } else {
      point = data.current_waypoint_index || 0; // <-- Key SUDAH BENAR (index berbasis 0)

      if (
        data.waypoints &&
        data.waypoints.length > 0 &&
        point < data.waypoints.length
      ) {
        point = point + 1; // Konversi ke berbasis 1 untuk tampilan
      } else {
        point = 0; // Default
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
    let currentLatLng = null;

    // === BACA data.latitude dan data.longitude ===
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
            // append current point to trail and update polyline
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
      if (data.speed !== undefined) {
        // Baca 'speed' (m/s)
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
        data.nav_heading_error !== undefined
          ? `${Math.round(parseFloat(data.nav_heading_error)) || 0}°`
          : "N/A";
    }

    // Update HARI, TANGGAL, dan WAKTU (menggunakan waktu LOKAL BROWSER)
    updateDateTime(elements);

    // --- MEMBUAT/MEMPERBARUI GARIS ARAH HDG (ORANGE) ---
    if (map && currentLatLng && currentHdg !== null && !isNaN(currentHdg)) {
      const distanceKm = 0.05; // 50 meter
      const endPoint = calculateDestinationPoint(
        currentLatLng[0],
        currentLatLng[1],
        currentHdg,
        distanceKm
      );
      const linePoints = [currentLatLng, [endPoint.lat, endPoint.lng]];

      if (!headingLine) {
        headingLine = L.polyline(linePoints, {
          color: "orange", // Warna garis jadi oranye
          weight: 2,
          dashArray: "5, 10",
        }).addTo(map);
      } else {
        headingLine.setLatLngs(linePoints);
        headingLine.setStyle({ color: "orange" }); // Pastikan warna tetap
      }
    } else if (headingLine) {
      // Hapus garis jika GPS atau HDG tidak valid
      map.removeLayer(headingLine);
      headingLine = null;
    }
    // --- AKHIR LOGIKA UPDATE UI ---
  };

  // Perbarui waktu lokal setiap detik
  setInterval(() => updateDateTime(elements), 1000);
}

// === FUNGSI HELPER (TIDAK BERUBAH) ===

/**
 * Memperbarui elemen DAY, DATE, dan TIME menggunakan waktu lokal browser.
 */
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

/**
 * Konversi GPS desimal ke format N/S/E/W.
 */
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

/**
 * Menghitung titik akhir GPS berdasarkan titik awal, arah, dan jarak.
 */
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

// === FUNGSI BARU: Handle Manual Capture dari IMG (tanpa backend) ===
async function handleManualCapture(camType) {
  // camType: "surface" atau "underwater"
  let imgEl = null;
  let btn = null;
  if (camType === "surface") {
    imgEl = document.querySelector('#cam1-container img');
    btn = document.getElementById("manual-capture-surface");
  } else if (camType === "underwater") {
    imgEl = document.querySelector('#cam2-container img');
    btn = document.getElementById("manual-capture-underwater");
  } else {
    alert("Jenis kamera tidak valid!");
    return;
  }
  if (!imgEl) {
    alert("Stream kamera tidak ditemukan!");
    return;
  }
  if (btn) {
    btn.disabled = true;
    btn.textContent = "Memotret...";
  }
  try {
    // Buat canvas dan draw image
    const canvas = document.createElement('canvas');
    canvas.width = imgEl.naturalWidth || imgEl.width;
    canvas.height = imgEl.naturalHeight || imgEl.height;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(imgEl, 0, 0, canvas.width, canvas.height);
    // Convert ke dataURL
    const dataURL = canvas.toDataURL('image/jpeg');
    // Tambahkan ke galeri
    addImageToGallery(dataURL, camType);
    // alert("Foto berhasil diambil!"); // DIHAPUS
  } catch (err) {
    alert("Gagal mengambil foto: " + err.message);
  }
  if (btn) {
    btn.disabled = false;
    btn.textContent = camType === "surface" ? "Potret Atas (S)" : "Potret Bawah (U)";
  }
}

// Fungsi untuk menambah gambar ke galeri
function addImageToGallery(dataURL, camType) {
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
  galleryImagesEl.prepend(img); // Tambahkan di atas
}