"""
Cerința 3.1 - Conectarea la CoppeliaSim și inspecția scenei.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main():
    # Conectare la serverul CoppeliaSim (localhost:23000)
    client = RemoteAPIClient()
    sim = client.require('sim')

    print("=== Conexiune stabilita ===")
    version = sim.getInt32Param(sim.intparam_program_version)
    print(f"Versiune CoppeliaSim (encoded): {version}")

    # --- Obținerea handle-urilor ---
    robot       = sim.getObject('/PioneerP3DX')
    left_motor  = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')

    sensors = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]

    print(f"\n=== Handle-uri obiecte ===")
    print(f"Robot:        {robot}")
    print(f"Motor stang:  {left_motor}")
    print(f"Motor drept:  {right_motor}")
    print(f"Senzori:      {sensors}")

    # --- Poziția inițială a robotului ---
    pos = sim.getObjectPosition(robot, sim.handle_world)
    print(f"\n=== Pozitia initiala a robotului ===")
    print(f"X={pos[0]:.3f} m,  Y={pos[1]:.3f} m,  Z={pos[2]:.3f} m")

    # --- Lectura inițială a tuturor senzorilor ---
    print(f"\n=== Lectura initiala senzori (detectat, distanta) ===")
    for i, sensor in enumerate(sensors):
        result, distance, *_ = sim.readProximitySensor(sensor)
        detected = bool(result)
        dist_str = f"{distance:.3f} m" if detected else "---"
        print(f"  Sensor[{i:2d}]: detectat={str(detected):<5},  distanta={dist_str}")


if __name__ == '__main__':
    main()