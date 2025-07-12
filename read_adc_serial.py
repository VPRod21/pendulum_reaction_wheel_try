import serial
import time
import matplotlib.pyplot as plt

# --- CONFIGURACIÓN ---
SERIAL_PORT = '/dev/ttyACM0'  # Cambia a tu puerto (ej. COM3 en Windows)
BAUDRATE = 115200
VREF = 3.3                    # Voltaje de referencia del ADC
ADC_RESOLUTION = 4095        # Resolución máxima de ADC de 12 bits

# --- LECTURA SERIAL ---
adc_data = []
voltages = []
reading = False

with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
    print("Esperando datos del ESP32...")

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if "--- INICIO DATOS ADC ---" in line:
            adc_data = []
            voltages = []
            reading = True
            print("Inicio de datos ADC detectado.")
            continue

        elif "--- FIN DATOS ADC ---" in line:
            print("Fin de datos ADC detectado.")
            break

        elif reading and line.isdigit():
            val = int(line)
            adc_data.append(val)
            voltages.append((val / ADC_RESOLUTION) * VREF)

# --- GUARDAR EN TXT ---
with open("adc_data.txt", "w") as f:
    for value in adc_data:
        f.write(f"{value}\n")

# --- GUARDAR EN CSV ---
with open("adc_data.csv", "w") as f:
    f.write("Index,ADC_Value,Voltage_V\n")
    for i, (adc_val, voltage) in enumerate(zip(adc_data, voltages)):
        f.write(f"{i},{adc_val},{voltage:.4f}\n")

# --- GRAFICAR ---
plt.plot(voltages)
plt.title("Señal ADC (Convertida a Voltaje)")
plt.xlabel("Muestra")
plt.ylabel("Voltaje [V]")
plt.grid(True)
plt.tight_layout()
plt.savefig("adc_plot.png")
plt.show()

print(f"{len(adc_data)} muestras guardadas en adc_data.txt, adc_data.csv y adc_plot.png")
