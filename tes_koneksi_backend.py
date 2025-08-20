import requests

url = "http://127.0.0.1:5000/status"

print(f"Mencoba menghubungi backend di {url}...")

try:
    # Coba hubungi backend dengan timeout 5 detik
    response = requests.get(url, timeout=5)

    # Periksa apakah respons sukses (kode 200)
    if response.status_code == 200:
        print("\n✅ BERHASIL! Koneksi ke backend sukses.")
        print("Data yang diterima:")
        print(response.json())
    else:
        print(f"\n❌ GAGAL! Backend merespons dengan status error: {response.status_code}")

except requests.exceptions.Timeout:
    print("\n❌ GAGAL! Koneksi timeout.")
    print("Backend berjalan tetapi tidak merespons cukup cepat.")

except requests.exceptions.RequestException as e:
    print(f"\n❌ GAGAL! Terjadi error koneksi.")
    print(f"Error: {e}")