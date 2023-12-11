BOOTLOADER_SIZE = 0x8000
BOOTLOADER_FILE = "bootloader.bin"

with open(BOOTLOADER_FILE, "rb") as f:
    raw_file = f.read()
    f.close()

pad_needed_in_bytes = BOOTLOADER_SIZE - len(raw_file)
padding = bytes([0xFF for _ in range(pad_needed_in_bytes)])

with open(BOOTLOADER_FILE, "wb") as f:
    f.write(raw_file + padding)