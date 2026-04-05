"""
Cerința 3.4 - Comportament reactiv simplu: oprire la obstacol.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

V_FORWARD     = 2.0    # rad/s - viteza de deplasare inainte
STOP_DISTANCE = 0.5    # metri - distanta la care robotul opreste
FRONT_SENSORS = [2, 3, 4, 5]  # indicii senzorilor frontali (ajustati dupa cerinta 3.3)
SENSOR_MAX    = 1.0    # metri - valoare returnata cand senzorul nu detecteaza nimic


def get_min_front_distance(sim, sensors, front_indices):
    """
    Returneaza distanta minima detectata de senzorii frontali.

    Args:
        sim: obiectul API CoppeliaSim.
        sensors: lista completa de handle-uri senzori.
        front_indices: lista indicilor senzorilor de monitorizat.

    Returns:
        float: distanta minima in metri (SENSOR_MAX daca nimic detectat).
    """
    min_dist = SENSOR_MAX
    for idx in front_indices:
        result, distance, *_ = sim.readProximitySensor(sensors[idx])
        if result and distance < min_dist:
            min_dist = distance
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
    print(f"Robot pornit. Se opreste la obstacol < {STOP_DISTANCE} m. (Ctrl+C pentru iesire)")

    try:
        while True:
            dist_front = get_min_front_distance(sim, sensors, FRONT_SENSORS)

            if dist_front < STOP_DISTANCE:
                # OPRIT: obstacol detectat prea aproape
                sim.setJointTargetVelocity(left_motor,  0.0)
                sim.setJointTargetVelocity(right_motor, 0.0)
                print(f"[STOP]   Obstacol la {dist_front:.3f} m  (prag: {STOP_DISTANCE} m)")
            else:
                # MERS INAINTE: drum liber
                sim.setJointTargetVelocity(left_motor,  V_FORWARD)
                sim.setJointTargetVelocity(right_motor, V_FORWARD)
                print(f"[MERS]   Distanta frontala minima: {dist_front:.3f} m")

            time.sleep(0.05)   # 20 Hz - frecventa buclei de control

    except KeyboardInterrupt:
        print("\nOprire manuala.")
    finally:
        sim.setJointTargetVelocity(left_motor,  0.0)
        sim.setJointTargetVelocity(right_motor, 0.0)
        sim.stopSimulation()


if __name__ == '__main__':
    main()