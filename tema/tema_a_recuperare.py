"""
Cerința 3.4 - Comportament reactiv simplu: oprire la obstacol.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from enum import Enum
import random as rnd

V_FORWARD     = 2.0    # rad/s - viteza de deplasare inainte
STOP_DISTANCE = 0.5    # metri - distanta la care robotul opreste
FRONT_SENSORS = [2, 3, 4, 5]  # indicii senzorilor frontali (ajustati dupa cerinta 3.3)
SENSOR_MAX    = 1.0    # metri - valoare returnata cand senzorul nu detecteaza nimic
SEED          = 1234 
V_ROT         = 3.14159 / 2 # PI / 2 [rad/s] ~ 90°


class RobotState(Enum):
    FORWARD = 0
    BACKWARD = 1
    TURNING = 2

def next_state(current_state : RobotState, dist_front : float) -> None:
    prev_state = current_state.value - 1

    # Daca nu exista obstacul in fata si calea este libera
    # continua deplasarea.
    # prev_state este utilizat, deoarece in momentul trecerii din
    # FORWARD in BACKWARD, robotul va merge inapoi 1 secunda, astfel
    # distanta dintre robot si obiect devine mai mare decat distanta
    # minima de oprire, deci la urmatoarea iteratie robotul se va 
    # intoarce in starea FORWARD, in locul starii ROTATE.
    # prev_state evita ciclul infinit: FORWARD <=> BACKWARD.
    
    if dist_front > STOP_DISTANCE and prev_state == -1:
        return RobotState.FORWARD
    
    if current_state == RobotState.FORWARD:
        return RobotState.BACKWARD
    
    if current_state == RobotState.BACKWARD:
        return RobotState.TURNING
    
    if current_state == RobotState.TURNING:
        return RobotState.FORWARD

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

    robot_state = RobotState.FORWARD
    rnd.seed(SEED)
    sim.startSimulation()
    print(f"Robot pornit. Se opreste la obstacol < {STOP_DISTANCE} m. (Ctrl+C pentru iesire)")

    try:
        while True:
            dist_front = get_min_front_distance(sim, sensors, FRONT_SENSORS)
            
            robot_state  = next_state(robot_state, dist_front)
            if robot_state == RobotState.FORWARD:
                # MERS INAINTE: drum liber
                sim.setJointTargetVelocity(left_motor,  V_FORWARD)
                sim.setJointTargetVelocity(right_motor, V_FORWARD)
                print(f"[MERS INAINTE]   Distanta frontala minima: {dist_front:.3f} m")
                # time.sleep(1)
            elif robot_state == RobotState.BACKWARD:
                # MERS INAPOI: drum blocat
                sim.setJointTargetVelocity(left_motor,  -V_FORWARD)
                sim.setJointTargetVelocity(right_motor, -V_FORWARD)
                print(f"[MERS INAPOI]   Distanta frontala minima: {dist_front:.3f} m")     
                time.sleep(1)
            else:
                # SCHIMBARE DIRECTIE: stanga / dreapta
                dir = rnd.randint(0, 1)
                sim.setJointTargetVelocity(left_motor,  V_ROT * dir)
                sim.setJointTargetVelocity(right_motor, V_ROT * (1 - dir))
                print(f"[SCHIMBARE DIRECTIE]   Distanta frontala minima: {dist_front:.3f} m.")
                time.sleep(1)

            time.sleep(0.05)   # 20 Hz - frecventa buclei de control

    except KeyboardInterrupt:
        print("\nOprire manuala.")
    finally:
        sim.setJointTargetVelocity(left_motor,  0.0)
        sim.setJointTargetVelocity(right_motor, 0.0)
        sim.stopSimulation()


if __name__ == '__main__':
    import os
    os.system("pwd")
    print(__file__)
    main()