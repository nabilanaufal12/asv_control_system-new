// js/firebase.js

function setupFirebase(config, elements) {
  // Memulai pengambilan data dan mengaturnya agar berjalan setiap 1 detik
  fetchData(config, elements);
  setInterval(() => fetchData(config, elements), 1000);
}

function fetchData(config, elements) {
  // Mengambil data dari URL Firebase yang ditentukan di file config
  fetch(config.FIREBASE_URL)
    .then((response) => {
      // Jika respons tidak ok, lemparkan error untuk ditangkap oleh .catch
      if (!response.ok) {
        throw new Error("Network response was not ok");
      }
      return response.json();
    })
    .then((data) => {
      // Jika tidak ada data, jangan lakukan apa-apa
      if (!data) return;

      // Memperbarui elemen HTML dengan data dari Firebase
      if (data.gps) {
        elements.gpsValue.textContent = `${data.gps.lat}, ${data.gps.lng}`;
      }

      elements.hdgValue.textContent =
        data.hdg !== undefined ? `${data.hdg}°` : "N/A";
      elements.sogValue.textContent =
        data.sog !== undefined ? `${data.sog} m/s` : "N/A";
      elements.cogValue.textContent =
        data.cog !== undefined ? `${data.cog}°` : "N/A";

      // Panggil fungsi untuk memperbarui hari dan tanggal
      updateDateTime(elements);
    })
    .catch((error) => {
      // Jika terjadi error (misalnya, tidak ada koneksi internet), tampilkan pesan di konsol
      console.error("Gagal mengambil data dari Firebase:", error);
    });
}

function updateDateTime(elements) {
  // Fungsi untuk menampilkan hari dan tanggal saat ini
  const now = new Date();
  const days = ["Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"];
  const options = { day: "2-digit", month: "2-digit", year: "numeric" };

  elements.dayValue.textContent = days[now.getDay()];
  elements.dateValue.textContent = now.toLocaleDateString("id-ID", options);
}
