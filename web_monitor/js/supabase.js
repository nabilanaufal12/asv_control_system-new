// js/supabase.js

let videoStream = null; // Variabel global untuk menyimpan stream kamera

function setupCamera(elements) {
  // Hanya menampilkan feed kamera, tanpa tombol potret
  async function startCamera() {
    try {
      videoStream = await navigator.mediaDevices.getUserMedia({
        video: { width: 640, height: 480 },
      });

      const videoElement = document.createElement("video");
      videoElement.srcObject = videoStream;
      videoElement.autoplay = true;
      videoElement.playsInline = true;
      videoElement.className = "camera-feed";

      elements.cam1Container.innerHTML = "";
      elements.cam1Container.appendChild(videoElement);
    } catch (error) {
      console.error("Camera error:", error);
      elements.cam1Container.innerHTML =
        '<div class="error">Kamera tidak dapat diakses</div>';
    }

    // Simulasikan kamera kedua
    elements.cam2Container.innerHTML =
      '<div class="camera-placeholder">CAM 2 FEED</div>';
  }

  startCamera();

  // Membersihkan stream saat halaman ditutup untuk mematikan kamera
  window.addEventListener("beforeunload", () => {
    if (videoStream) {
      videoStream.getTracks().forEach((track) => track.stop());
    }
  });
}

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
