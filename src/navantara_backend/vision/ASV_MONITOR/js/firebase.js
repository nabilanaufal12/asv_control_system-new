// js/firebase.js

let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 }; // Menyimpan posisi GPS terakhir

function setupFirebase(config, elements) {
  fetchData(config, elements);
  setInterval(() => fetchData(config, elements), 1000);
}

function fetchData(config, elements) {
  fetch(config.FIREBASE_URL)
    .then((response) => {
      if (!response.ok) {
        throw new Error("Network response was not ok");
      }
      return response.json();
    })
    .then((data) => {
      if (!data) return;

      // Kontrol Arena
      if (data.arena && data.arena !== lastKnownArena) {
        lastKnownArena = data.arena;
        const switchArenaEvent = new CustomEvent("switchArena", {
          detail: { arena: data.arena },
        });
        window.dispatchEvent(switchArenaEvent);
      }

      // Kontrol Titik (Point)
      if (data.point !== undefined && data.point !== lastKnownPoint) {
        lastKnownPoint = data.point;
        const setPointEvent = new CustomEvent("setTrajectoryPoint", {
          detail: { point: data.point },
        });
        window.dispatchEvent(setPointEvent);
      }
      
      // --- PEMBARUAN PETA GPS ---
      if (data.gps && (data.gps.lat !== lastKnownGps.lat || data.gps.lng !== lastKnownGps.lng)) {
        // Simpan koordinat baru
        lastKnownGps.lat = data.gps.lat;
        lastKnownGps.lng = data.gps.lng;

        // Perbarui teks GPS di "Voyage Information"
        elements.gpsValue.textContent = `${data.gps.lat}, ${data.gps.lng}`;
        
        // Buat URL Google Maps baru dengan zoom level 18 (lebih dekat)
        const newMapUrl = `https://maps.google.com/maps?q=${data.gps.lat},${data.gps.lng}&hl=id&z=18&output=embed`;

        // Ganti sumber iframe peta
        if (elements.mapIframe) {
            elements.mapIframe.src = newMapUrl;
        }
      }
      // ----------------------------

      // Memperbarui elemen HTML lainnya
      elements.hdgValue.textContent =
        data.hdg !== undefined ? `${data.hdg}°` : "N/A";
      elements.sogValue.textContent =
        data.sog !== undefined ? `${data.sog} m/s` : "N/A";
      elements.cogValue.textContent =
        data.cog !== undefined ? `${data.cog}°` : "N/A";

      updateDateTime(elements);
    })
    .catch((error) => {
      console.error("Gagal mengambil data dari Firebase:", error);
    });
}

function updateDateTime(elements) {
  const now = new Date();
  const days = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"];
  const options = { day: "2-digit", month: "2-digit", year: "numeric" };
  elements.dayValue.textContent = days[now.getDay()];
  elements.dateValue.textContent = now.toLocaleDateString("id-ID", options);
}