import numpy as np
import matplotlib.pyplot as plt

LUT_SIZE = 4095

# ----- Generate waveform LUTs -----
# Sine LUT (12-bit)
sine_lut = [int(2048 + 2047*np.sin(2*np.pi*i/LUT_SIZE)) for i in range(LUT_SIZE)]

# Sawtooth LUT (0 -> 4095)
saw_lut = [int(i*(4095/(LUT_SIZE-1))) for i in range(LUT_SIZE)]

# Triangle LUT
triangle_lut = [int(i*(4095/(LUT_SIZE//2 -1)) if i < LUT_SIZE//2 else 4095 - (i - LUT_SIZE//2)*(4095/(LUT_SIZE//2 -1))) for i in range(LUT_SIZE)]

# Drum LUT you provided


# ----- Print LUTs as C arrays -----
print("uint16_t sineLUT[NS] = {", ", ".join(map(str, sine_lut)), "};\n")
print("uint16_t sawLUT[NS] = {", ", ".join(map(str, saw_lut)), "};\n")
print("uint16_t triangleLUT[NS] = {", ", ".join(map(str, triangle_lut)), "};\n")


# ----- Plot all LUTs -----
plt.figure(figsize=(12,8))

plt.subplot(4,1,1)
plt.plot(sine_lut, marker='o')
plt.title("Sine Wave LUT")
plt.grid(True)

plt.subplot(4,1,2)
plt.plot(saw_lut, marker='o', color='orange')
plt.title("Sawtooth Wave LUT")
plt.grid(True)

plt.subplot(4,1,3)
plt.plot(triangle_lut, marker='o', color='green')
plt.title("Triangle Wave LUT")
plt.grid(True)



plt.tight_layout()
plt.show()
