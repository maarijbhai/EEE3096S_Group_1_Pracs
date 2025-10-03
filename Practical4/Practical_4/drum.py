import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt

# Load wav file
fs, data = wavfile.read("drum.wav")

# If stereo, take one channel
if data.ndim > 1:
    data = data[:, 0]

# Normalize to -1 to 1
data = data / np.max(np.abs(data))

# Scale to 12-bit DAC range (0–4095)
data = (data + 1) * 2047.5  # shift from [-1,1] to [0,4095]

# Downsample to LUT size
lut_size = 10000
indices = np.linspace(0, len(data) - 1, lut_size, dtype=int)
lut = data[indices].astype(int)

# Clip to safe DAC range
lut = np.clip(lut, 0, 4095)

# Plot to check
plt.plot(lut)
plt.title("LUT from wav file")
plt.show()

# Print out as proper C array with line wrapping
array_name = "Guitar_LUT"
values_per_line = 12  # how many values to print per line

print(f"const uint16_t {array_name}[{lut_size}] = {{")
for i in range(0, lut_size, values_per_line):
    line = ", ".join(map(str, lut[i:i+values_per_line]))
    print("  " + line + ",")
print("};")
