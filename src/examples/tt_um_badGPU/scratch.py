
import numpy as np

from scipy import signal
import matplotlib.pyplot as plt

# print((np.geomspace(1, 32, 4, dtype=int)))

sos1 = signal.butter(N=1, Wn=21000, fs=44000, btype='low', output='sos')
sos2 = signal.butter(N=2, Wn=21000, fs=44000, btype='low', output='sos')
sos3 = signal.butter(N=3, Wn=21000, fs=44000, btype='low', output='sos')

print("First order filter coefficients:")
print(sos1)
print("Second order filter coefficients:")
print(sos2)
print("Third order filter coefficients:")
print(sos3)


w1, h1 = signal.sosfreqz(sos=sos1, fs=44000)
w2, h2 = signal.sosfreqz(sos=sos2, fs=44000)
w3, h3 = signal.sosfreqz(sos=sos3, fs=44000)

plt.plot(w1, abs(h1), label='1st order filter', color='blue')
plt.plot(w2, abs(h2), label='2nd order filter', color='green')
plt.plot(w3, abs(h3), label='3rd order filter', color='red')

plt.legend()
plt.show()

    # while True:
    #     print(adc.read_u16() * 5.03477e-5)
    #     time.sleep_us(50)


    # N_SAMPLE = 80
    # MINUS_TWO_J_PI = -2j * cmath.pi / N_SAMPLE

    # adc = machine.ADC(machine.Pin(GPIOMapTT06.RPIO29))

    # data = [0] * N_SAMPLE
    # processed_vals = [0] * N_SAMPLE

    # while True:
    #     t_start = time.ticks_us()

    #     # Collect data
    #     for sample in range(N_SAMPLE):
    #         data[sample] = adc.read_u16()
    #         time.sleep_us(50)

    #     t_start = time.ticks_us()

    #     # Process data
    #     for bin in range(N_SAMPLE):
    #         val = 0
    #         for sample in range(N_SAMPLE):
    #             val += data[sample] * cmath.exp(bin * sample * MINUS_TWO_J_PI)
    #         processed_vals[bin] = cmath.sqrt(val.real**2 + val.imag**2).real

    #     t_end = time.ticks_us()

    #     # Some delays to allow the print outputs to format nicely
    #     print(f"Time taken: {(t_end - t_start) * 1e-6} seconds")
    #     time.sleep(0.5)
    #     print(f"Processed values: {processed_vals}")
    #     time.sleep(0.5)
    #     print()
    #     print()

    # arr = np.zeros(64)

    # while True:

    #     for sample in range(64):
    #         arr[sample] = adc.read_u16()
    #         time.sleep_us(50)

    #     real = ulab.utils.spectrogram(arr)