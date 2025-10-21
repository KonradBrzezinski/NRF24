import serial
import matplotlib.pyplot as plt
import math

# --- konfiguracja portu szeregowego ---
ser = serial.Serial(
    port="COM7",       # zmień jeśli masz inny port
    baudrate=115200,   # dopasuj do ustawień swojego urządzenia
    timeout=1
)

# --- tablice na dane ---
xs = []
ys = []
y2s = []

# --- filtr dolnoprzepustowy (średnia ruchoma) ---
def low_pass_simple(new_value, prev_values, window_size=5):
    prev_values.append(new_value)
    if len(prev_values) > window_size:
        prev_values.pop(0)
    return sum(prev_values) / len(prev_values)

ys_filtered_window = []

# --- wczytywanie danych z portu ---
print("Rozpoczynam odczyt danych. Naciśnij Ctrl+C, aby przerwać...")
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
                # y2 = float(parts[2])

                # filtr dolnoprzepustowy
                # y_filtered = low_pass_simple(y, ys_filtered_window, window_size=5)

                xs.append(x)
                # ys.append(y_filtered)
                # ys.append
                # y2s.append(y2)
                ys.append(y)
                if(x == 499):
                    print("Odczytano 500 próbek, kończę...")
                    break
except KeyboardInterrupt:
    print("Odczyt przerwany przez użytkownika.")

finally:
    ser.close()
    print("Port szeregowy zamknięty.")

# --- wyświetlanie wykresu po zakończeniu odczytu ---
plt.figure(figsize=(10, 5))
plt.plot(xs, ys, "b-")
# plt.plot(xs, y2s, "r-")
plt.title("Wykres X-Y po zakończeniu odczytu")
plt.xlabel("X")
plt.ylabel("Y (filtrowane)")
plt.grid(True)
plt.show()
