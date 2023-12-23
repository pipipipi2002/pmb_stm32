BOOTLOADER_SIZE = 0x8000
BOOTLOADER_FILE = "bootloader.bin"

with open(BOOTLOADER_FILE, "rb") as f:
    raw_file = f.read()
    f.close()

pad_needed_in_bytes = BOOTLOADER_SIZE - len(raw_file)
padding = bytes([0xFF for _ in range(pad_needed_in_bytes)])

with open(BOOTLOADER_FILE, "wb") as f:
    f.write(raw_file + padding)
    f.close()

with open(BOOTLOADER_FILE, "rb") as f:
    check_file = f.read()
    f.close

if (len(check_file) != BOOTLOADER_SIZE):
    raise Exception(f"Bootloader size not padded properly. \n\t Size aft padding: {len(check_file)}.  \n\t Target size: {BOOTLOADER_SIZE}")