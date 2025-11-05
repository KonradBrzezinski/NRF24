import serial
import matplotlib.pyplot as plt
import math
from scipy.signal import butter, filtfilt
import numpy as np

# --- konfiguracja portu szeregowego ---
ser = serial.Serial(
    port="COM6",       # zmień jeśli masz inny port
    baudrate=115200,   # dopasuj do ustawień swojego urządzenia
    timeout=1
)

# --- tablice na dane ---
xs = []
ys = []  # po filtrze średniej ruchomej

# --- filtr dolnoprzepustowy (średnia ruchoma) ---
def low_pass_simple(new_value, prev_values, window_size=5):
    prev_values.append(new_value)
    if len(prev_values) > window_size:
        prev_values.pop(0)
    return sum(prev_values) / len(prev_values)

ys_filtered_window = []

# --- wczytywanie danych z portu ---
print("Rozpoczynam odczyt danych...")
try:
    while True:
        linia = ser.readline().decode(errors="ignore").strip()
        if linia:
            if "," in linia:
                parts = linia.split(",")
            else:
                parts = linia.split()

            if len(parts) >= 2:
                x = float(parts[0])
                y = abs(float(parts[1]))

                # filtr dolnoprzepustowy (średnia ruchoma online)
                y_filtered = low_pass_simple(y, ys_filtered_window, window_size=5)

                xs.append(x)
                ys.append(y_filtered)

                if x == 3599:  # koniec pomiaru
                    print("Odczytano 3600 próbek, kończę...")
                    break
except KeyboardInterrupt:
    print("Odczyt przerwany przez użytkownika.")

finally:
    ser.close()
    print("Port szeregowy zamknięty.")

# --- dodatkowy filtr Butterwortha (offline) ---
def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    return filtfilt(b, a, data)

# parametry filtra
fs = 3600        # zakładam, że 3600 próbek to 1 sekunda -> fs=3600 Hz
cutoff = 10      # częstotliwość odcięcia w Hz
order = 4        # rząd filtra

ys_butter = butter_lowpass_filter(ys, cutoff, fs, order)

# --- wyświetlanie wykresu po zakończeniu odczytu ---
plt.figure(figsize=(10, 5))
plt.plot(xs, ys, "b-", label="Po średniej ruchomej")
plt.plot(xs, ys_butter, "r-", label="Po Butterworth LPF")
plt.title("Wykres X-Y po zakończeniu odczytu (z filtracją)")
plt.xlabel("X")
plt.ylabel("Y (filtrowane)")
plt.legend()
plt.grid(True)
plt.show()
