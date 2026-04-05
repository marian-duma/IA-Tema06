"""
Cerința 3.6 - Wall-following: urmărirea peretelui din dreapta.
IA Lab #06 - Inteligență Artificială 2025-2026
"""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from enum import Enum
import matplotlib.pyplot as plt

V_BASE       = 2.0    # rad/s - viteza de baza
TARGET_DIST  = 0.4    # metri - distanta dorita fata de peretele drept
K_P          = 3    # constanta proportionala
K_I          = 0.01   # constanta integratoare
K_D          = 1.2    # constanta derivatoare
FRONT_STOP   = 0.4    # metri - distanta de declansare viraj la obstacol frontal
SENSOR_MAX   = 1.0    # metri - valoare implicita cand senzorul nu detecteaza

RIGHT_SENSORS = [8, 9]   # senzori laterali dreapta
FRONT_SENSORS = [3, 4]   # senzori frontali pentru detectare obstacol

ROBOT_WIDTH = 0.45  # Lățimea robotului în metri
SAFETY_MARGIN = 0.1 # Spațiu suplimentar de siguranță
MIN_PASSAGE = ROBOT_WIDTH + SAFETY_MARGIN

# Variabile de stare pentru PID
last_error = 0
integral = 0

class RobotState(Enum):
    FORWARD = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2

def next_state(dist_front : float, dist_right : float) -> None:
    if dist_front > FRONT_STOP and dist_right <= SENSOR_MAX * 0.95:
        return RobotState.FORWARD
    if dist_front > FRONT_STOP and dist_right >= SENSOR_MAX * 0.95:
        return RobotState.TURN_RIGHT
    if dist_front < FRONT_STOP:
        return RobotState.TURN_LEFT

def calculate_pid(current_dist, target_dist, dt):
    global last_error, integral
    
    # 1. Calculăm eroarea
    error = current_dist - target_dist
    
    # 2. Termenul Proporțional
    P = K_P * error
    
    # 3. Termenul Integral (cu protecție la "windup")
    integral += error * dt
    I = K_I * integral
    
    # 4. Termenul Derivativ (viteza de schimbare a erorii)
    derivative = (error - last_error) / dt
    D = K_D * derivative
    
    # Salvează eroarea pentru iterația următoare
    last_error = error
    
    return P + I + D
    
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
    global last_error, integral
    client = RemoteAPIClient()
    sim = client.require('sim')

    left_motor  = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')
    robot = sim.getObject('/PioneerP3DX')
    pos : list[tuple[float, float]] = []
    sensors     = [
        sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]')
        for i in range(16)
    ]
    
    robot_state = RobotState.FORWARD
    sim.startSimulation()
    print(f"Wall-following pornit. Distanta tinta perete drept: {TARGET_DIST} m")
    print("(Ctrl+C pentru oprire)\n")

    try:
        while True:
            dist_right = read_min_dist(sim, sensors, RIGHT_SENSORS)
            dist_front = read_min_dist(sim, sensors, FRONT_SENSORS)
            
            robot_state = next_state(dist_front, dist_right)
            
            if robot_state == RobotState.FORWARD:
                correction = calculate_pid(dist_right, TARGET_DIST, 0.05)

                v_left = V_BASE + correction
                v_right = V_BASE - correction
                
                # Limitare la [-V_BASE*1.5, +V_BASE*1.5]
                cap = V_BASE * 1.5
                v_left  = max(-cap, min(cap, v_left))
                v_right = max(-cap, min(cap, v_right))
            elif robot_state == RobotState.TURN_LEFT:
                v_left, v_right = -V_BASE, +V_BASE
                integral = 0
                last_error = 0
            else:
                v_left, v_right = V_BASE, V_BASE * 0.5
                integral = 0
                last_error = 0
              
            sim.setJointTargetVelocity(left_motor,  v_left)
            sim.setJointTargetVelocity(right_motor, v_right)

            x, y, _ = sim.getObjectPosition(robot, -1)

            pos.append((x,y))
            time.sleep(0.05)   # 20 Hz

    except KeyboardInterrupt:
        print("\nOprire wall-follower.")
    finally:
        try:
            sim.setJointTargetVelocity(left_motor,  0.0)
            sim.setJointTargetVelocity(right_motor, 0.0)
            sim.stopSimulation()
        except Exception as e:
            print(f"Notă: Conexiunea cu simulatorul a fost întreruptă ({e})")
        
        trajectory_x = [p[0] for p in pos]
        trajectory_y = [p[1] for p in pos]

        plt.figure(figsize=(10, 8))
        plt.plot(trajectory_x, trajectory_y, label='Traiectorie Robot', color='blue', linewidth=2)
        
        
        if trajectory_x:
            plt.scatter(trajectory_x[0], trajectory_y[0], color='green', s=100, label='START', zorder=5)
            plt.scatter(trajectory_x[-1], trajectory_y[-1], color='red', s=100, label='FINAL', zorder=5)

        plt.title(f'Traiectoria Robotului Pioneer P3DX (Wall-Following)\nParams: K_P={K_P}, K_D={K_D}, V={V_BASE}')
        plt.xlabel('Poziție X (metri)')
        plt.ylabel('Poziție Y (metri)')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        plt.axis('equal')

        import os
        plt.savefig(os.path.dirname(os.path.abspath(__file__)) + '/traiectorie_robot.png')
        print("Graficul a fost salvat ca 'traiectorie_robot.png'.")
        
        plt.show()


if __name__ == '__main__':
    main()