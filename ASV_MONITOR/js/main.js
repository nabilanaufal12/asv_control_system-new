// js/main.js

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
    // --- PERUBAHAN DI SINI ---
    mapIframe: document.getElementById("map-iframe"), // Menambahkan elemen peta
  };

  setupTabs(ELEMENTS);
  setupFirebase(CONFIG, ELEMENTS);
  setupCamera(ELEMENTS);
  renderGallery(CONFIG, ELEMENTS);
  setupModal(ELEMENTS);
});

function setupTabs(elements) {
  elements.tabButtons.forEach((button) => {
    button.addEventListener("click", () => {
      elements.tabButtons.forEach((btn) => btn.classList.remove("active"));
      button.classList.add("active");
      const tabName = button.getAttribute("data-tab");
      elements.surfaceCaptures.classList.toggle("hidden", tabName !== "surface");
      elements.underwaterCaptures.classList.toggle("hidden", tabName !== "underwater");
    });
  });
}