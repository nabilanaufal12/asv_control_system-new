import torch

print("Versi Torch:", torch.__version__)
print("Versi CUDA Torch:", torch.version.cuda)
print("CUDA tersedia? ", torch.cuda.is_available())

if torch.cuda.is_available():
    print("Jumlah GPU:", torch.cuda.device_count())
    print("Nama GPU:", torch.cuda.get_device_name(0))
else:
    print("❌ CUDA tidak tersedia, fallback ke CPU")
