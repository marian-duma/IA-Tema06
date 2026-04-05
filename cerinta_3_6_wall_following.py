"""
Cerința 3.6 - Wall-following: urmărirea peretelui din dreapta.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

V_BASE       = 2.0    # rad/s - viteza de baza
TARGET_DIST  = 0.4    # metri - distanta dorita fata de peretele drept
K_P          = 3.0    # coeficient proportional (P-controller)
FRONT_STOP   = 0.4    # metri - distanta de declansare viraj la obstacol frontal
SENSOR_MAX   = 1.0    # metri - valoare implicita cand senzorul nu detecteaza

RIGHT_SENSORS = [8, 9]   # senzori laterali dreapta
FRONT_SENSORS = [3, 4]   # senzori frontali pentru detectare obstacol


def read_min_dist(sim, sensors, indices):
    """
    Returneaza distanta minima detectata de un grup de senzori.

    Args:
        sim: obiectul API CoppeliaSim.
        sensors: lista completa de handle-uri senzori.
        indices: lista indicilor senzorilor de verificat.

    Returns:
        float: distanta minima in metri.
    """
    min_dist = SENSOR_MAX
    for idx in indices:
        result, dist, *_ = sim.readProximitySensor(sensors[idx])
        if result and dist < min_dist:
            min_dist = dist
    return min_dist


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    left_motor  = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')
    sensors     = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]

    sim.startSimulation()
    print(f"Wall-following pornit. Distanta tinta perete drept: {TARGET_DIST} m")
    print("(Ctrl+C pentru oprire)\n")

    try:
        while True:
            dist_right = read_min_dist(sim, sensors, RIGHT_SENSORS)
            dist_front = read_min_dist(sim, sensors, FRONT_SENSORS)

            if dist_front < FRONT_STOP:
                # Obstacol frontal: viraj la stanga
                v_left, v_right = -V_BASE, +V_BASE
                state = f"VIREAZA STANGA (frontal={dist_front:.3f} m)"

            elif dist_right >= SENSOR_MAX * 0.95:
                # Nu exista perete la dreapta: cautam perete (viraj usor dreapta)
                v_left, v_right = V_BASE, V_BASE * 0.5
                state = "CAUTA PERETE (viraj dreapta)"

            else:
                # Controller P: eroare = distanta actuala - tinta
                error   = dist_right - TARGET_DIST
                v_left  = V_BASE + K_P * error
                v_right = V_BASE - K_P * error

                # Limitare la [-V_BASE*1.5, +V_BASE*1.5]
                cap = V_BASE * 1.5
                v_left  = max(-cap, min(cap, v_left))
                v_right = max(-cap, min(cap, v_right))

                state = f"URMARIRE  dr={dist_right:.3f} m  err={error:+.3f} m"

            sim.setJointTargetVelocity(left_motor,  v_left)
            sim.setJointTargetVelocity(right_motor, v_right)
            print(f"{state:<45}  vS={v_left:+.2f}  vD={v_right:+.2f}")

            time.sleep(0.05)   # 20 Hz

    except KeyboardInterrupt:
        print("\nOprire wall-follower.")
    finally:
        sim.setJointTargetVelocity(left_motor,  0.0)
        sim.setJointTargetVelocity(right_motor, 0.0)
        sim.stopSimulation()


if __name__ == '__main__':
    main()