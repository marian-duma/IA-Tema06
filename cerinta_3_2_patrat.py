"""
Cerința 3.2 - Controlul motorului: mișcare în pătrat (buclă deschisă).
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- Parametri de mișcare (ajustați experimental dacă e nevoie) ---
V_FORWARD = 2.0    # rad/s - viteza înainte (ambele roți egale)
V_TURN    = 2.0    # rad/s - viteza pentru viraj (pe o singură roată)
T_LINIE   = 3.0    # secunde - timp mers drept (o latură de pătrat)
T_VIRAJ   = 1.57   # secunde - timp viraj ~90° (π/2 rad, empiric)


def set_velocity(sim, left_motor, right_motor, v_left, v_right):
    """
    Setează viteza țintă a ambelor motoare.

    Args:
        sim: obiectul API CoppeliaSim.
        left_motor: handle-ul motorului stâng.
        right_motor: handle-ul motorului drept.
        v_left: viteza roată stângă (rad/s).
        v_right: viteza roată dreaptă (rad/s).
    """
    sim.setJointTargetVelocity(left_motor,  v_left)
    sim.setJointTargetVelocity(right_motor, v_right)


def move_forward(sim, left_motor, right_motor, duration):
    """Mișcare rectilinie pentru 'duration' secunde."""
    set_velocity(sim, left_motor, right_motor, V_FORWARD, V_FORWARD)
    time.sleep(duration)


def turn_left_90(sim, left_motor, right_motor):
    """Viraj stânga ~90°: roata dreaptă înainte, roata stângă înapoi."""
    set_velocity(sim, left_motor, right_motor, -V_TURN, V_TURN)
    time.sleep(T_VIRAJ)


def stop(sim, left_motor, right_motor):
    """Oprire completă a ambelor motoare."""
    set_velocity(sim, left_motor, right_motor, 0.0, 0.0)


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    robot       = sim.getObject('/PioneerP3DX')
    left_motor  = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')

    sim.startSimulation()
    print("Simulare pornita. Robotul va parcurge un patrat.")
    time.sleep(0.5)  # pauza scurta de stabilizare

    try:
        for latura in range(4):
            print(f"Latura {latura + 1}/4 - mers inainte {T_LINIE}s")
            move_forward(sim, left_motor, right_motor, T_LINIE)

            print(f"Viraj stanga ~90°")
            turn_left_90(sim, left_motor, right_motor)

        stop(sim, left_motor, right_motor)

        pos = sim.getObjectPosition(robot, sim.handle_world)
        print(f"\nPozitie finala: X={pos[0]:.3f} m,  Y={pos[1]:.3f} m")
        print("(Ideal: robotul sa fie aproape de pozitia initiala 0, 0)")
        time.sleep(1.0)

    finally:
        stop(sim, left_motor, right_motor)
        sim.stopSimulation()
        print("Simulare oprita.")


if __name__ == '__main__':
    main()