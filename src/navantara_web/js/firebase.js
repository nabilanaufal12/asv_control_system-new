// js/firebase.js

// --- 1. INISIALISASI FIREBASE ---
// GANTI DENGAN KONFIGURASI FIREBASE ANDA YANG SEBENARNYA
const firebaseConfig = {
  apiKey: "AIzaSy...", // GANTI INI
  authDomain: "asv-2025.firebaseapp.com", // GANTI INI
  databaseURL: "https://asv-2025-default-rtdb.asia-southeast1.firebasedatabase.app", // PASTIKAN BENAR
  projectId: "asv-2025", // GANTI INI
  storageBucket: "asv-2025.appspot.com", // GANTI INI
  messagingSenderId: "...", // GANTI INI
  appId: "1:..." // GANTI INI
};

// Inisialisasi Firebase
if (typeof firebase !== 'undefined' && !firebase.apps.length) {
   firebase.initializeApp(firebaseConfig);
} else if (typeof firebase === 'undefined') {
    console.error("Firebase SDK belum dimuat. Pastikan script SDK ada di HTML sebelum file ini.");
}
// --- AKHIR INISIALISASI ---


// --- 2. FUNGSI SETUP FIREBASE ---
let lastKnownArena = null;
let lastKnownPoint = 0;
let lastKnownGps = { lat: 0, lng: 0 };

function setupFirebase(config, elements) {
  fetchData(config, elements);
  setInterval(() => fetchData(config, elements), 1000);
}

function fetchData(config, elements) {
  fetch(config.FIREBASE_URL)
    .then((response) => {
      if (!response.ok) {
        throw new Error(`Network response was not ok: ${response.statusText}`);
      }
      return response.json();
    })
    .then((data) => {
      if (!data) {
          console.warn("Data Firebase null.");
          if (headingLine) {
              // console.log("Menghapus garis HDG karena data Firebase null.");
              map.removeLayer(headingLine);
              headingLine = null;
          }
          return;
      }

      // Kontrol Arena
      if (data.arena && data.arena !== lastKnownArena) {
        lastKnownArena = data.arena;
        const switchArenaEvent = new CustomEvent("switchArena", { detail: { arena: data.arena } });
        window.dispatchEvent(switchArenaEvent);
      }

      // Kontrol Titik (Point)
      if (data.point !== undefined && data.point !== lastKnownPoint) {
        lastKnownPoint = data.point;
        const setPointEvent = new CustomEvent("setTrajectoryPoint", { detail: { point: data.point } });
        window.dispatchEvent(setPointEvent);
      }

      // --- PEMBARUAN PETA GPS & GARIS HDG ---
      let currentLatLng = null;
      if (data.gps && (data.gps.lat !== lastKnownGps.lat || data.gps.lng !== lastKnownGps.lng)) {
        lastKnownGps.lat = data.gps.lat;
        lastKnownGps.lng = data.gps.lng;

        if (elements.gpsValue) {
            try {
                const lat = parseFloat(data.gps.lat);
                const lng = parseFloat(data.gps.lng);
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
        if (data.hdg !== undefined) {
            try {
                const hdgNum = parseFloat(data.hdg);
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

      // Update SOG (km/jam dan knots)
       if (elements.sogValue) {
        if (data.sog !== undefined) {
          try {
            // Data 'sog' dari Firebase sudah dalam km/jam
            const sog_kmh_raw = parseFloat(data.sog);
            if (isNaN(sog_kmh_raw)) throw new Error("SOG bukan angka");

            const sog_kmh = sog_kmh_raw.toFixed(1); // Tampilkan nilai km/jam
            const sog_knots = (sog_kmh_raw * 0.539957).toFixed(1); // Konversi km/jam ke knots
            
            elements.sogValue.textContent = `${sog_kmh} km/jam (${sog_knots} kn)`;
          } catch (e) {
            console.error("Data SOG tidak valid:", e);
            elements.sogValue.textContent = "N/A";
          }
        } else {
          elements.sogValue.textContent = "N/A";
        }
      }
      // Update COG
       if (elements.cogValue) {
        elements.cogValue.textContent =
          data.cog !== undefined ? `${Math.round(parseFloat(data.cog)) || 0}°` : "N/A"; // Bulatkan COG
      }

      // Update TIME dari Firebase
      if (elements.timeValue) {
        elements.timeValue.textContent =
          data.jam !== undefined ? data.jam : "N/A";
      }

      // Update HARI dan TANGGAL (waktu lokal)
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
          // console.log(`Menghapus garis HDG. Alasan: currentLatLng=${currentLatLng}, currentHdg=${currentHdg}`);
          map.removeLayer(headingLine);
          headingLine = null;
      }
      // --- AKHIR BAGIAN GARIS HDG ---

    })
    .catch((error) => {
      console.error("Gagal mengambil data dari Firebase:", error);
       if (headingLine) {
           // console.log("Menghapus garis HDG karena fetch gagal.");
           map.removeLayer(headingLine);
           headingLine = null;
       }
    });
}

// Fungsi updateDateTime (Hanya untuk DAY dan DATE lokal)
function updateDateTime(elements) {
  const now = new Date();
  // Mengubah array 'days' ke Bahasa Inggris
  const days = ["Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"];
  const options = { day: "2-digit", month: "2-digit", year: "numeric" };
  
  if (elements.dayValue) {
    elements.dayValue.textContent = days[now.getDay()];
  }
  if (elements.dateValue) {
    // Anda mungkin juga ingin mengubah lokalisasi tanggal ke 'en-GB' (DD/MM/YYYY) atau 'en-US' (MM/DD/YYYY)
    // Di sini saya ubah ke 'en-GB' agar formatnya mirip (DD/MM/YYYY)
    elements.dateValue.textContent = now.toLocaleDateString("en-GB", options);
  }
}

// Fungsi konversi GPS
function decimalToHemisphere(dec, isLng) {
    let absDec = Math.abs(dec);
    let hem;
    if (isLng) { hem = dec >= 0 ? 'E' : 'W'; }
    else { hem = dec >= 0 ? 'N' : 'S'; }
    return `${hem} ${absDec.toFixed(5)}`;
}

// Fungsi calculateDestinationPoint
function calculateDestinationPoint(lat1, lon1, bearing, distanceKm) {
    const R = 6371; // Radius bumi dalam km
    const bearingRad = bearing * Math.PI / 180;
    const lat1Rad = lat1 * Math.PI / 180;
    const lon1Rad = lon1 * Math.PI / 180;
    const lat2Rad = lat1Rad + (distanceKm / R) * Math.cos(bearingRad);
    const lon2Rad = lon1Rad + (distanceKm / R / Math.cos(lat1Rad)) * Math.sin(bearingRad);
    const lat2 = lat2Rad * 180 / Math.PI;
    const lon2 = lon2Rad * 180 / Math.PI;
    return { lat: lat2, lng: lon2 };
}