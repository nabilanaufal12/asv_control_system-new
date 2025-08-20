import torch
print(torch.cuda.is_available())   # harus True
print(torch.cuda.get_device_name(0))  # harus keluar nama GPU NVIDIA
