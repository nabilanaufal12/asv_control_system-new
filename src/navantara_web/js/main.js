// js/main.js

// Variabel Global untuk Peta Leaflet
let map;
let vehicleMarker;
let headingLine = null;

document.addEventListener("DOMContentLoaded", () => {
  const CONFIG = {
    FIREBASE_URL:
      "https://asv-2025-default-rtdb.asia-southeast1.firebasedatabase.app/monitor.json",
    SUPABASE_URL: "https://ngmicoyombtgtlegdjzn.supabase.co",
    SUPABASE_BUCKET: "navantara",
    SUPABASE_TOKEN:
      "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Im5nbWljb3lvbWJ0Z3RsZWdkanpuIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTM5NjkwOTMsImV4cCI6MjA2OTU0NTA5M30.tz7_mfVaX_5sWO3xtSQ1h85JAIRrL6iC4ZNm0TaUTdI",
  };

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

  if (typeof setupFirebase === 'function') {
    setupFirebase(CONFIG, ELEMENTS);
  } else {
    console.error("Fungsi setupFirebase tidak ditemukan.");
  }

  // Panggil renderGallery saat halaman dimuat
  if (typeof renderGallery === 'function') {
    renderGallery(CONFIG, ELEMENTS);
  } else {
    console.error("Fungsi renderGallery tidak ditemukan.");
  }

  // Panggil setupModal
  if (typeof setupModal === 'function') {
    setupModal(ELEMENTS);
  } else {
    console.warn("Fungsi setupModal tidak ditemukan.");
  }

  // Tambahkan Event Listener untuk tombol Refresh Galeri
  if (ELEMENTS.refreshGalleryBtn) {
      ELEMENTS.refreshGalleryBtn.addEventListener('click', () => {
          console.log("Refresh button clicked. Fetching gallery...");
          if (typeof renderGallery === 'function') {
              renderGallery(CONFIG, ELEMENTS);
              // Pastikan renderGallery mengosongkan galeri sebelum mengisi ulang
          } else {
               console.error("renderGallery function not found.");
          }
      });
  }
});

// Fungsi setupTabs Anda
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