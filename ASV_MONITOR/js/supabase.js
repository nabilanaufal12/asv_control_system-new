// file: ASV_MONITOR/js/supabase.js

let videoStream = null; // Variabel global untuk menyimpan stream kamera

// --- MODIFIKASI DIMULAI ---

function setupCamera(elements) {
  // Ganti IP '127.0.0.1' dengan IP asli Jetson Anda
  // jika Anda mengakses monitor dari perangkat lain (misal: laptop).
  // Jika Anda membuka monitor di Jetson itu sendiri, '127.0.0.1' sudah benar.
  const JETSON_IP = "127.0.0.1"; 
  const STREAM_URL = `http://${JETSON_IP}:5000/live_video_feed`;

  // Hentikan penggunaan webcam lokal (getUserMedia)
  // dan langsung buat elemen <img> untuk menampilkan stream.
  
  // Setup untuk CAM 1 (Stream Utama)
  try {
    const imgElement = document.createElement("img");
    imgElement.src = STREAM_URL;
    imgElement.alt = "Memuat stream dari ASV...";
    imgElement.style.width = "100%";
    imgElement.style.height = "100%";
    imgElement.style.objectFit = "cover";
    
    // Tampilkan error jika stream gagal dimuat
    imgElement.onerror = () => {
      elements.cam1Container.innerHTML =
        '<div class="error">Gagal memuat stream. Pastikan backend berjalan dan IP benar.</div>';
    };
    
    elements.cam1Container.innerHTML = "";
    elements.cam1Container.appendChild(imgElement);

  } catch (error) {
    console.error("Gagal memulai stream:", error);
    elements.cam1Container.innerHTML =
      '<div class="error">Error saat setup stream.</div>';
  }

  // Simulasikan kamera kedua (seperti kode asli)
  elements.cam2Container.innerHTML =
    '<div class="camera-placeholder">CAM 2 FEED (WebSocket)</div>';
  
  // Catatan: Kode stream WebSocket untuk cam2 dari GUI utama 
  // (api_client.py) belum diimplementasikan di monitor ini.
  // Monitor ini sekarang hanya menampilkan stream utama via HTTP.
}

// --- MODIFIKASI SELESAI ---


async function renderGallery(config, elements) {
  try {
    elements.surfaceCaptures.innerHTML = "Memuat gambar surface...";
    elements.underwaterCaptures.innerHTML = "Memuat gambar underwater...";

    // Mengambil daftar gambar untuk kedua kategori secara paralel
    const [surfaceImages, underwaterImages] = await Promise.all([
      fetchImages("surface", config),
      fetchImages("underwater", config),
    ]);

    displayImages(surfaceImages, elements.surfaceCaptures, elements);
    displayImages(underwaterImages, elements.underwaterCaptures, elements);
  } catch (error) {
    console.error("Gallery error:", error);
    elements.surfaceCaptures.innerHTML =
      '<div class="error">Gagal memuat gambar</div>';
    elements.underwaterCaptures.innerHTML =
      '<div class="error">Gagal memuat gambar</div>';
  }
}

async function fetchImages(type, config) {
  const images = [];
  const maxCheck = 20; // Jumlah maksimum gambar yang akan diperiksa per kategori

  for (let i = 1; i <= maxCheck; i++) {
    const imageName = `${type}_${i}.jpg`;
    const imageUrl = `${config.SUPABASE_URL}/storage/v1/object/public/${config.SUPABASE_BUCKET}/${imageName}`;

    // Memeriksa apakah gambar ada sebelum menambahkannya ke daftar
    const exists = await checkImageExists(imageUrl);
    if (exists) {
      images.push({ url: imageUrl, name: imageName });
    }
  }

  return images;
}

function checkImageExists(url) {
  // Fungsi helper untuk memastikan URL gambar valid dan bisa dimuat
  return new Promise((resolve) => {
    const img = new Image();
    img.onload = () => resolve(true);
    img.onerror = () => resolve(false);
    img.src = url;
  });
}

function displayImages(images, container, elements) {
  if (images.length === 0) {
    container.innerHTML = '<div class="no-images">Tidak ada foto</div>';
    return;
  }

  container.innerHTML = ""; // Kosongkan kontainer sebelum mengisi

  images.forEach((image) => {
    const item = document.createElement("div");
    item.className = "capture-item";

    const img = document.createElement("img");
    img.src = image.url;
    img.alt = image.name;
    img.loading = "lazy"; // Optimasi: gambar hanya dimuat saat akan terlihat

    // Tambahkan event listener untuk membuka modal saat gambar diklik
    img.addEventListener("click", () => {
      openModal(image.url, image.name, elements);
    });

    item.appendChild(img);
    container.appendChild(item);
  });
}

// Fungsi-fungsi untuk modal pratinjau dan unduh
function setupModal(elements) {
  elements.closeModalBtn.addEventListener("click", () => closeModal(elements));

  window.addEventListener("click", (event) => {
    if (event.target === elements.modal) {
      closeModal(elements);
    }
  });
}

function openModal(imageUrl, imageName, elements) {
  elements.modalImg.src = imageUrl;
  elements.downloadBtn.href = imageUrl;
  elements.downloadBtn.download = imageName;
  elements.modal.style.display = "flex";
}

function closeModal(elements) {
  elements.modal.style.display = "none";
}