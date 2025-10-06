import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt

# Settings
wav_files = ["piano.wav", "drum.wav", "guitar.wav"]
lut_size = 4095

for wav_file in wav_files:
    # Load wav file
    fs, data = wavfile.read(wav_file)

    # If stereo, take one channel
    if data.ndim > 1:
        data = data[:, 0]

    # Normalize to -1 to 1
    data = data / np.max(np.abs(data))

    # Scale to 12-bit DAC range (0–4095)
    data = (data + 1) * 2047.5  # shift from [-1,1] to [0,4095]

    # Downsample to LUT size
    indices = np.linspace(0, len(data) - 1, lut_size, dtype=int)
    lut = data[indices].astype(int)

    # Clip to safe DAC range
    lut = np.clip(lut, 0, 4095)

    # Plot to check
    plt.plot(lut)
    plt.title(f"LUT from {wav_file}")
    plt.show()

    # Create array name from filename
    array_name = wav_file.split(".")[0].capitalize() + "_LUT"

    # Print out as proper C array (no line wrapping)
    print(f"const uint16_t {array_name}[NS] = {{")
    print(", ".join(map(str, lut)) + ",")
    print("};\n")
