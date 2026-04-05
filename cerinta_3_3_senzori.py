"""
Cerința 3.3 - Citirea și vizualizarea în timp real a senzorilor de proximitate.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
import os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

SENSOR_MAX_RANGE = 1.0   # metri - raza maxima de detectare a senzorilor

# Etichete descriptive pentru cei 16 senzori (orientative, validati prin experiment)
SENSOR_LABELS = [
    "S00  fata-stanga-ext ",
    "S01  fata-stanga     ",
    "S02  fata-centru-st  ",
    "S03  fata-centru-st  ",
    "S04  fata-centru-dr  ",
    "S05  fata-centru-dr  ",
    "S06  fata-dreapta    ",
    "S07  fata-dreapta-ext",
    "S08  lateral-dreapta ",
    "S09  lateral-dreapta ",
    "S10  spate-dreapta   ",
    "S11  spate-centru    ",
    "S12  spate-centru    ",
    "S13  spate-stanga    ",
    "S14  lateral-stanga  ",
    "S15  lateral-stanga  ",
]


def read_all_sensors(sim, sensors):
    """
    Citeste toti senzorii si returneaza lista de lecturi.

    Args:
        sim: obiectul API CoppeliaSim.
        sensors: lista de handle-uri ale senzorilor.

    Returns:
        Lista de tupluri (bool detectat, float distanta_m).
    """
    readings = []
    for sensor in sensors:
        result, distance, *_ = sim.readProximitySensor(sensor)
        detected = bool(result)
        dist = distance if detected else SENSOR_MAX_RANGE
        readings.append((detected, dist))
    return readings


def print_dashboard(readings):
    """Afisaza un dashboard text cu lecturile tuturor senzorilor."""
    os.system('cls' if os.name == 'nt' else 'clear')
    print("=== DASHBOARD SENZORI ULTRASONICI - Pioneer P3-DX ===\n")
    print(f"  {'Idx':<5} {'Eticheta':<25} {'Detectat':<10} {'Distanta':>10}  {'Bar':}")
    print("  " + "-" * 65)
    for i, (detected, dist) in enumerate(readings):
        bar_len = int((1.0 - dist / SENSOR_MAX_RANGE) * 20) if detected else 0
        bar     = "█" * bar_len
        dist_str = f"{dist:.3f} m" if detected else "(nimic)"
        det_str  = "DA" if detected else "nu"
        print(f"  [{i:2d}]  {SENSOR_LABELS[i]}  {det_str:<10} {dist_str:>10}  {bar}")
    print("\n  Ctrl+C pentru oprire.")


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sensors = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]

    print("Monitorizare senzori pornita (simularea trebuie sa fie activa).")
    print("Plasati sau mutati un obiect in CoppeliaSim si observati schimbarile.\n")

    try:
        while True:
            readings = read_all_sensors(sim, sensors)
            print_dashboard(readings)
            time.sleep(0.5)   # actualizare la fiecare 0.5 secunde
    except KeyboardInterrupt:
        print("\nMonitorizare oprita.")


if __name__ == '__main__':
    main()