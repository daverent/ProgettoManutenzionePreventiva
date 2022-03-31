from math import sqrt
from time import sleep
from pymavlink import mavutil
import csv

class FlyVector:
    def __init__(
        self,
        timestamp=0,
        x=0,
        y=0,
        z=0,
        vx=0,
        vy=0,
        vz=0,
        p=0,
        q=0,
        r=0,
        phi=0,
        theta=0,
        psi=0,
        uf=0,
        up=0,
        uq=0,
        ur=0
    ):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.p = p
        self.q = q
        self.r = r
        self.phi = phi
        self.theta = theta
        self.psi = psi
        self.uf = uf
        self.up = up
        self.uq = uq
        self.ur = ur


def initializeConnection():
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection("udpin:localhost:14551")

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print(
        "Heartbeat from system (system %u component %u)"
        % (the_connection.target_system, the_connection.target_component)
    )

    the_connection.mav.request_data_stream_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1000,
        1,
    )

    return the_connection

def getFlyVector(connection):
    servos_output = connection.recv_match(type="SERVO_OUTPUT_RAW", blocking=True)
    rcou1, rcou2, rcou3, rcou4 = (
        servos_output.servo1_raw,
        servos_output.servo2_raw,
        servos_output.servo3_raw,
        servos_output.servo4_raw,
    )
    cmd1 = (rcou1-1000)/(1950-1000)
    cmd2 = (rcou2-1000)/(1950-1000)
    cmd3 = (rcou3-1000)/(1950-1000)
    cmd4 = (rcou4-1000)/(1950-1000)

    MAX_RPM = 10200
    kt = 1.077e-5
    kq = 9.596e-8
    l = 0.25

    omegamax_squared = (MAX_RPM/60*2*3.1416)**2

    omega1_squared = cmd1 * omegamax_squared 
    omega2_squared = cmd2 * omegamax_squared
    omega3_squared = cmd3 * omegamax_squared
    omega4_squared = cmd4 * omegamax_squared

    uf = kt*(omega1_squared + omega2_squared + omega3_squared + omega4_squared)
    up = (sqrt(2)/2)*kt*l*(-omega1_squared+omega2_squared+omega3_squared-omega4_squared)
    uq = (sqrt(2)/2)*kt*l*(omega1_squared-omega2_squared+omega3_squared-omega4_squared)
    ur = kq*(omega1_squared+omega2_squared-omega3_squared-omega4_squared)


    positions = connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    x, y, z, vx, vy, vz = (
        positions.x,
        positions.y,
        positions.z,
        positions.vx,
        positions.vy,
        positions.vz,
    )

    attitude = connection.recv_match(type="ATTITUDE", blocking=True)
    # p, q, r, vp, vq, vr = (
    #     attitude.roll,
    #     attitude.pitch,
    #     attitude.yaw,
    #     attitude.rollspeed,
    #     attitude.pitchspeed,
    #     attitude.yawspeed,
    # )

    p, q, r = ( #velocità angolari
        attitude.rollspeed,
        attitude.pitchspeed,
        attitude.yawspeed
        )

    ahrs2 = connection.recv_match(type="AHRS2", blocking=True)

    phi, theta, psi = ( #angoli
        ahrs2.roll,
        ahrs2.pitch,
        ahrs2.yaw
        )

    timestamp = attitude.time_boot_ms

    fly_vector = FlyVector(
        timestamp,
        x,
        y,
        z,
        vx,
        vy,
        vz,
        p,
        q,
        r,
        phi,
        theta,
        psi,
        uf,
        up,
        uq,
        ur
    )
    return fly_vector

def main():

    connection = initializeConnection()

    fly_data = [FlyVector()]

    outfile = open("test_input", mode='w')
    csvwriter = csv.writer(outfile)

    csvwriter.writerow(['TimeStamp','X', 'Y', 'Z', 'VX', 'VY', 'VZ', 'P', 'Q', 'R', 'Phi', 'Theta', 'Psi', 'UF', 'UP', 'UQ', 'UR'])
    
    
    while True:  # freq max 111hz (sync ogni 9ms)

        fly_vector = getFlyVector(connection)
        fly_data.append(fly_vector)

        csvwriter.writerow([fly_vector.timestamp,
                           fly_vector.x, 
                           fly_vector.y,
                           fly_vector.z,
                           fly_vector.vx, 
                           fly_vector.vy, 
                           fly_vector.vz, 
                           fly_vector.p,
                           fly_vector.q,
                           fly_vector.r,
                           fly_vector.phi, 
                           fly_vector.theta, 
                           fly_vector.psi,
                           fly_vector.uf, 
                           fly_vector.up, 
                           fly_vector.uq, 
                           fly_vector.ur])
    
        sleep(0.001)

if __name__ == "__main__":
    main()