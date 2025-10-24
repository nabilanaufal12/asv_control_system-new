// file: src/navantara_backend/ASV_MONITOR/js/supabase.js

let videoStream = null; // Variabel global (meskipun tidak digunakan lagi untuk stream)

function setupCamera(elements) {
  // --- KONFIGURASI IP DAN URL ---
  // Ganti IP '127.0.0.1' dengan IP asli Jetson jika diakses dari perangkat lain.
  const JETSON_IP = "127.0.0.1";
  const STREAM_URL_CAM1 = `http://${JETSON_IP}:5000/live_video_feed`;       // Endpoint untuk CAM 1 (Hasil AI)
  const STREAM_URL_CAM2 = `http://${JETSON_IP}:5000/live_video_feed_cam2`; // Endpoint BARU untuk CAM 2 (Mentah)
  // -----------------------------


  // --- Setup untuk CAM 1 (Stream Utama - Hasil AI) ---
  try {
    const imgElement1 = document.createElement("img");
    imgElement1.src = STREAM_URL_CAM1;
    imgElement1.alt = "Memuat stream CAM 1...";
    imgElement1.style.width = "100%";
    imgElement1.style.height = "100%";
    imgElement1.style.objectFit = "cover"; // Pastikan gambar menutupi area

    // Tampilkan error jika stream CAM 1 gagal dimuat
    imgElement1.onerror = () => {
      elements.cam1Container.innerHTML =
        '<div class="error">Gagal memuat stream CAM 1. Pastikan backend berjalan dan IP benar.</div>';
    };

    elements.cam1Container.innerHTML = ""; // Kosongkan container dulu
    elements.cam1Container.appendChild(imgElement1); // Tambahkan elemen img

  } catch (error) {
    console.error("Gagal memulai stream CAM 1:", error);
    elements.cam1Container.innerHTML =
      '<div class="error">Error saat setup stream CAM 1.</div>';
  }


  // --- Setup untuk CAM 2 (Stream Mentah) ---
  try {
    const imgElement2 = document.createElement("img");
    imgElement2.src = STREAM_URL_CAM2; // Gunakan URL endpoint CAM 2
    imgElement2.alt = "Memuat stream CAM 2...";
    imgElement2.style.width = "100%";
    imgElement2.style.height = "100%";
    imgElement2.style.objectFit = "cover"; // Pastikan gambar menutupi area

    // Tampilkan error jika stream CAM 2 gagal dimuat
    imgElement2.onerror = () => {
      elements.cam2Container.innerHTML =
        '<div class="error">Gagal memuat stream CAM 2. Pastikan kamera terhubung & backend berjalan.</div>';
    };

    elements.cam2Container.innerHTML = ""; // Kosongkan placeholder CAM 2
    elements.cam2Container.appendChild(imgElement2); // Tambahkan elemen img ke container CAM 2

  } catch (error) {
    console.error("Gagal memulai stream CAM 2:", error);
    elements.cam2Container.innerHTML =
      '<div class="error">Error saat setup stream CAM 2.</div>';
  }
}


// --- Fungsi Galeri (Tidak berubah) ---
async function renderGallery(config, elements) {
  try {
    elements.surfaceCaptures.innerHTML = "Memuat gambar surface...";
    elements.underwaterCaptures.innerHTML = "Memuat gambar underwater...";

    const [surfaceImages, underwaterImages] = await Promise.all([
      fetchImages("surface", config),
      fetchImages("underwater", config),
    ]);

    displayImages(surfaceImages, elements.surfaceCaptures, elements);
    displayImages(underwaterImages, elements.underwaterCaptures, elements);
  } catch (error) {
    console.error("Gallery error:", error);
    elements.surfaceCaptures.innerHTML =
      '<div class="error">Gagal memuat gambar surface</div>';
    elements.underwaterCaptures.innerHTML =
      '<div class="error">Gagal memuat gambar underwater</div>';
  }
}

async function fetchImages(type, config) {
  const images = [];
  const maxCheck = 20; // Cek hingga 20 gambar per kategori

  for (let i = 1; i <= maxCheck; i++) {
    const imageName = `${type}_${i}.jpg`;
    const imageUrl = `${config.SUPABASE_URL}/storage/v1/object/public/${config.SUPABASE_BUCKET}/${imageName}`;

    const exists = await checkImageExists(imageUrl);
    if (exists) {
      images.push({ url: imageUrl, name: imageName });
    }
  }
  return images;
}

function checkImageExists(url) {
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
  container.innerHTML = ""; // Kosongkan sebelum mengisi

  images.forEach((image) => {
    const item = document.createElement("div");
    item.className = "capture-item";
    const img = document.createElement("img");
    img.src = image.url;
    img.alt = image.name;
    img.loading = "lazy";
    img.addEventListener("click", () => openModal(image.url, image.name, elements));
    item.appendChild(img);
    container.appendChild(item);
  });
}

// --- Fungsi Modal (Tidak berubah) ---
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
  elements.modal.style.display = "flex"; // Gunakan flex untuk centering
}

function closeModal(elements) {
  elements.modal.style.display = "none";
}