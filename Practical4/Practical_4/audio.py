
import numpy as np
from scipy.io import wavfile
from scipy.signal import resample
import matplotlib.pyplot as plt

# === SETTINGS ===
wave_files = ["piano.wav", "guitar.wav", "drum.wav"]  # Input .wav files
num_samples = 4095        # Number of samples per LUT
target_fs = 44100         # Target sample rate (Hz)
bits = 12                 # 12-bit DAC resolution
NS = "NS"                 # Symbolic array size for C definition

# === FUNCTION TO GENERATE LUT FROM WAV FILE ===
def generate_lut(filename):
    fs, data = wavfile.read(filename)

    # If stereo, take one channel
    if data.ndim > 1:
        data = data[:, 0]

    # Normalize to range [-1, 1]
    data = data / np.max(np.abs(data))

    # Resample if necessary
    if fs != target_fs:
        duration = len(data) / fs
        num_target_samples = int(duration * target_fs)
        data = resample(data, num_target_samples)

    # Select evenly spaced points for the LUT
    indices = np.linspace(0, len(data) - 1, num_samples, dtype=int)
    lut = data[indices]

    # Scale to 12-bit range (0–4095)
    lut_scaled = ((lut + 1) / 2) * (2**bits - 1)
    lut_scaled = np.round(lut_scaled).astype(int)

    return lut_scaled

# === PROCESS EACH WAVEFORM ===
for filename in wave_files:
    lut = generate_lut(filename)
    array_name = filename.split('.')[0]  # e.g. 'sine', 'saw', 'triangle'

    # Plot waveform
    plt.figure(figsize=(8, 3))
    plt.plot(lut)
    plt.title(f"{array_name} waveform LUT ({num_samples} samples)")
    plt.xlabel("Sample Index")
    plt.ylabel("Amplitude (0–4095)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Print entire array in one line
    lut_str = ", ".join(str(v) for v in lut)
    print(f"uint16_t {array_name}_LUT[{NS}] = {{ {lut_str} }};")
    print()  # Blank line between arrays
