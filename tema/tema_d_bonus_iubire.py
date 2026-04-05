import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

V_BASE = 2.5
SENSOR_MAX = 1.0


LEFT_SENSORS  = [1, 2, 3] 
RIGHT_SENSORS = [4, 5, 6]

def read_intensity(sim, sensors, indices):
    """
    Citește distanța minimă și o transformă în intensitate a stimulului [0, 1].
    0 = nimic detectat, 1 = coliziune iminentă (distanța aproape 0).
    """
    min_dist = SENSOR_MAX
    for idx in indices:
        res, dist, *_ = sim.readProximitySensor(sensors[idx])
        if res and dist < min_dist:
            min_dist = dist
            
    # Dacă dist = SENSOR_MAX -> intensity = 0.0
    # Dacă dist = 0.0 -> intensity = 1.0
    intensity = max(0.0, 1.0 - (min_dist / SENSOR_MAX))
    return intensity

def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    left_motor  = sim.getObject('/PioneerP3DX/leftMotor')
    right_motor = sim.getObject('/PioneerP3DX/rightMotor')
    sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(16)]
    
    sim.startSimulation()
    print("Vehiculul 'Iubire' a pornit. Se va opri blând în fața obstacolelor.")
    
    try:
        while True:
            # 1. Calculăm intensitatea stimulului pe ambele părți
            i_left  = read_intensity(sim, sensors, LEFT_SENSORS)
            i_right = read_intensity(sim, sensors, RIGHT_SENSORS)

            # 2. Conexiuni Ipsilaterale Inhibitorii
            # Intensitatea stângă scade viteza roții stângi
            # Intensitatea dreaptă scade viteza roții drepte
            v_left  = V_BASE * (1.0 - i_left)
            v_right = V_BASE * (1.0 - i_right)

            # 3. Trimitem comanda la motoare
            sim.setJointTargetVelocity(left_motor, v_left)
            sim.setJointTargetVelocity(right_motor, v_right)

            time.sleep(0.05) # 20 Hz

    except KeyboardInterrupt:
        pass
    finally:
        sim.setJointTargetVelocity(left_motor, 0)
        sim.setJointTargetVelocity(right_motor, 0)
        sim.stopSimulation()
        print("Simulare oprită.")

if __name__ == '__main__':
    main()
