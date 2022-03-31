from pymavlink import mavutil
import numpy as np
import math
from math import sqrt
import control
import control.matlab as signal
from reprint import output
from time import sleep

from sqlite3 import Timestamp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class FlyVector:
    """
    Classe per la struttura dati che raccolga gli stati di volo.
    """
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

class GraphVector:
    """
    Classe per la struttura dati che raccolga gli stati di volo per la stampa dei grafici.
    """
    TIME=[]
    UF=[]
    UP=[]
    UQ=[]
    UR=[]
    X=[]
    Y=[]
    Z=[]
    Vx=[]
    Vy=[]
    Vz=[]
    P=[]
    Q=[]
    R=[]
    Phi=[]
    Theta=[]
    Psi=[]
    XOSS=[]
    YOSS=[]
    ZOSS=[]
    VxOSS=[]
    VyOSS=[]
    VzOSS=[]
    POSS=[]
    QOSS=[]
    ROSS=[]
    PhiOSS=[]
    ThetaOSS=[]
    PsiOSS=[]

    def update(self, fly_vector, residui):
        """
        Metodo per l'aggiornamento della struttura dati.
        """
        self.TIME.append(fly_vector.timestamp)
        self.UF.append(fly_vector.uf)
        self.UP.append(fly_vector.up)
        self.UQ.append(fly_vector.uq)
        self.UR.append(fly_vector.ur)
        self.X.append(fly_vector.x)
        self.Y.append(fly_vector.y)
        self.Z.append(fly_vector.z)
        self.Vx.append(fly_vector.vx)
        self.Vy.append(fly_vector.vy)
        self.Vz.append(fly_vector.vz)
        self.P.append(fly_vector.p)
        self.Q.append(fly_vector.q)
        self.R.append(fly_vector.r)
        self.Phi.append(fly_vector.phi)
        self.Theta.append(fly_vector.theta)
        self.Psi.append(fly_vector.psi)

        self.XOSS.append(residui[0])
        self.YOSS.append(residui[1])
        self.ZOSS.append(residui[2])
        self.VxOSS.append(residui[3])
        self.VyOSS.append(residui[4])
        self.VzOSS.append(residui[5])
        self.POSS.append(residui[6])
        self.QOSS.append(residui[7])
        self.ROSS.append(residui[8])
        self.PhiOSS.append(residui[9])
        self.ThetaOSS.append(residui[10])
        self.PsiOSS.append(residui[11])

def initializeConnection(): 
    """
    La funzione ha lo scopo di inizializzare la connessione con MAVProxy sfruttando la porta udp:14551.
    Ritorna una variabile di tipo mavutil.mavlink_connection.
    """
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
    """
    La funzione effettua il crawling dei dati di volo a partire dai messaggi forniti sulla connessione attraverso i metodi recv_match. 
    Il type dei metodi definisce il messaggio di stato da cui vengono raccolti i dati.

    Ritorna un vettore con i dati di volo attuali.
    """
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
        -positions.z, #Allineamento nomenclatura
        positions.vx,
        positions.vy,
        -positions.vz #Allineamento nomenclatura
    )

    attitude = connection.recv_match(type="ATTITUDE", blocking=True)

    p, q, r = ( #velocità angolari
        attitude.rollspeed,
        attitude.pitchspeed,
        attitude.yawspeed
        )

    ahrs2 = connection.recv_match(type="AHRS2", blocking=True)

    phi, theta, psi = ( #angoli accelerometro
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

def printGraphs(graph_vector):
    """
    Funzione di stampa dei grafici. Stampa tre finestre indicizzate.
    """
    
    #PLOT SENSOR INFO
    plt.figure(figsize=(12, 12))

    a1=plt.subplot(3, 2, 1)
    a1.set_title("Posizione nel tempo")
    a1.plot(graph_vector.TIME, graph_vector.X, color = 'red', label="Posizione X")    
    a1.plot(graph_vector.TIME, graph_vector.Y, color = 'green', label="Posizione Y")
    a1.plot(graph_vector.TIME, graph_vector.Z, color = 'blue', label="Posizione Z") 
    a1.legend(loc="lower right", frameon=False)

    a2=plt.subplot(3, 2, 2)
    a2.set_title("Velocità nel tempo")
    a2.plot(graph_vector.TIME, graph_vector.Vx, color = 'red', label="Velocità X")    
    a2.plot(graph_vector.TIME, graph_vector.Vy, color = 'green', label="Velocità Y")
    a2.plot(graph_vector.TIME, graph_vector.Vz, color = 'blue', label="Velocità Z") 
    a2.legend(loc="lower right", frameon=False)

    a3=plt.subplot(3, 2, 3)
    a3.set_title("Velocità angolare nel tempo")
    a3.plot(graph_vector.TIME, graph_vector.P, color = 'red', label="Velocità angolare P")    
    a3.plot(graph_vector.TIME, graph_vector.Q, color = 'green', label="Velocità angolare Q")
    a3.plot(graph_vector.TIME, graph_vector.R, color = 'blue', label="Velocità angolare R") 
    a3.legend(loc="lower right", frameon=False)

    a4=plt.subplot(3, 2, 4)
    a4.set_title("Angoli nel tempo")
    a4.plot(graph_vector.TIME, graph_vector.Phi, color = 'red', label="Angolo Phi")    
    a4.plot(graph_vector.TIME, graph_vector.Theta, color = 'green', label="Angolo Theta")
    a4.plot(graph_vector.TIME, graph_vector.Psi, color = 'blue', label="Angolo Psi") 
    a4.legend(loc="lower right", frameon=False)

    a5=plt.subplot(3, 2, 5)
    a5.set_title("U nel tempo")
    a5.plot(graph_vector.TIME, graph_vector.UF, color = 'red', label="UF")    
    a5.plot(graph_vector.TIME, graph_vector.UP, color = 'green', label="UP")
    a5.plot(graph_vector.TIME, graph_vector.UQ, color = 'blue', label="UQ") 
    a5.plot(graph_vector.TIME, graph_vector.UR, color = 'yellow', label="UR") 
    a5.legend(loc="lower right", frameon=False)

    plt.show()

    #TRAIETTORIA 3D
    plt.title("Posizione nello spazio") 
    ax = plt.axes(projection='3d')
    ax.scatter3D(graph_vector.X, graph_vector.Y, graph_vector.Z, c=graph_vector.Z, cmap='Greens')

    plt.show()  

    #PLOT RESIDUI
    plt.figure(figsize=(12, 12))

    a1=plt.subplot(5, 3, 1)
    a1.plot(graph_vector.TIME, graph_vector.XOSS, color = 'red', label="Residuo X")    
    a1.legend(loc="lower right", frameon=False)

    a2=plt.subplot(5, 3, 2)
    a2.plot(graph_vector.TIME, graph_vector.YOSS, color = 'red', label="Residuo Y")    
    a2.legend(loc="lower right", frameon=False)

    a3=plt.subplot(5, 3, 3)
    a3.plot(graph_vector.TIME, graph_vector.ZOSS, color = 'red', label="Residuo Z")    
    a3.legend(loc="lower right", frameon=False)

    a4=plt.subplot(5, 3, 4)
    a4.plot(graph_vector.TIME, graph_vector.VxOSS, color = 'red', label="Residuo Velocità X")    
    a4.legend(loc="lower right", frameon=False)

    a5=plt.subplot(5, 3, 5)
    a5.plot(graph_vector.TIME, graph_vector.VyOSS, color = 'red', label="Residuo Velocità Y")    
    a5.legend(loc="lower right", frameon=False)

    a6=plt.subplot(5, 3, 6)
    a6.plot(graph_vector.TIME, graph_vector.VzOSS, color = 'red', label="Residuo Velocità Z")    
    a6.legend(loc="lower right", frameon=False)

    a7=plt.subplot(5, 3, 7)
    a7.plot(graph_vector.TIME, graph_vector.POSS, color = 'red', label="Residuo Velocità Angolare P")    
    a7.legend(loc="lower right", frameon=False)

    a8=plt.subplot(5, 3, 8)
    a8.plot(graph_vector.TIME, graph_vector.QOSS, color = 'red', label="Residuo Velocità Angolare Q")    
    a8.legend(loc="lower right", frameon=False)

    a9=plt.subplot(5, 3, 9)
    a9.plot(graph_vector.TIME, graph_vector.ROSS, color = 'red', label="Residuo Velocità Angolare R")    
    a9.legend(loc="lower right", frameon=False)

    a10=plt.subplot(5, 3, 10)
    a10.plot(graph_vector.TIME, graph_vector.PhiOSS, color = 'red', label="Residuo Angolo Phi")    
    a10.legend(loc="lower right", frameon=False)

    a11=plt.subplot(5, 3, 11)
    a11.plot(graph_vector.TIME, graph_vector.ThetaOSS, color = 'red', label="Residuo Angolo Theta")    
    a11.legend(loc="lower right", frameon=False)

    a12=plt.subplot(5, 3, 12)
    a12.plot(graph_vector.TIME, graph_vector.PsiOSS, color = 'red', label="Residuo Angolo Psi")    
    a12.legend(loc="lower right", frameon=False)

    ResOr=[]
    for i in range(len(graph_vector.XOSS)):
        ResOr.append(math.sqrt(pow(graph_vector.VxOSS[i],2)+(pow(graph_vector.VyOSS[i],2))))

    a13=plt.subplot(5, 3, 13)
    a13.plot(graph_vector.TIME, ResOr, color = 'red', label="Residuo Velocità Orizzontale")    
    a13.legend(loc="lower right", frameon=False)

    plt.show()

def main():
    """
    Funzione principale. Esegue osservazione.
    """
    fly_data = [FlyVector()] #Vettore di accumulazione dei dati di volo
    graph_vector = GraphVector() #Vettore di accumulazione dei dati per i grafici

    connection = initializeConnection()

    ######################################################################################
    """
    Definizione dell'osservatore in rappresentazione a spazio di stato.
    Le matrici A0, B0, C0, D0 sono calcolate con l'utilizzo di strumenti esterni e qui riportate.
    Consultare la documentazione allegata in formato pdf per ulteriori info.
    """
    A0 = np.diag([-4, -4.8, -4.2, -4.3, -4.5, -4.9, -5, -4.1, -4.4, -4.75, -4.25, -4.55])
    B0 = np.matrix([[0, 0, 0, 0, 4, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], 
                    [0, 0, 0, 0, 0, 4.8, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 4.2, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 4.2992, 0, 0, 0, 0, 0, 0, 9.81, 0], 
                    [0, 0, 0, 0, 0, 0, 0, 0, 4.4992, 0, 0, 0, 0, -9.81, 0, 0], 
                    [0.8197, 0, 0, 0, 0, 0, 0, 0, 0, 4.8992, 0, 0, 0, 0, 0, 0], 
                    [0, 122.6392, 0, 0, 0, 0, 0, 0, 0, 0, 4.8774, 0, 0, 0, 0, 0], 
                    [0, 0, 122.6392, 0, 0, 0, 0, 0, 0, 0, 0, 3.9774, 0, 0, 0, 0],
                    [0, 0, 0, 57.2803, 0, 0, 0, 0, 0, 0, 0, 0, 4.3427, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 4.75, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 4.25, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 4.55]])
    C0 = np.identity(12)
    D0 = np.zeros((12, 16))

    Obs = control.ss(A0, B0, C0, D0)
    """
    Utilizzo della libreria Python Control per la definizione di un sistema LTI in spazio di stato
    """
    ######################################################################################

    print("Lo script ha bisogno di essere interrotto manualmente.\nPremi ctrl+c quando desideri terminare l'osservazione.\n\n")

    with output(output_type='dict') as output_lines:
        """
        Avvio delle condizioni di output.
        """
        ######################################################################################
        """
        Osservazione iniziale a partire dallo stato 0 nullo.
        """
        fly_vector = getFlyVector(connection)

        u_y = np.array([ #Generazione ingresso
                fly_vector.uf,
                fly_vector.up,
                fly_vector.uq,
                fly_vector.ur,
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
                fly_vector.psi
            ])

        y_oss, _, x_ossprec = signal.lsim(Obs, U=np.array([np.zeros(16), u_y]), T=np.linspace(0, fly_vector.timestamp, num=2, endpoint=True)) #Calcolo prima evoluzione dallo stato iniziale

        Xe = fly_vector.x - y_oss[-1][0]
        Ye = fly_vector.y - y_oss[-1][1]
        Ze = fly_vector.z - y_oss[-1][2]
        vXe = fly_vector.vx - y_oss[-1][3]
        vYe = fly_vector.vy - y_oss[-1][4]
        vZe = fly_vector.vz - y_oss[-1][5]
        Pe = fly_vector.p - y_oss[-1][6]
        Qe = fly_vector.q - y_oss[-1][7]
        Re = fly_vector.r - y_oss[-1][8]
        Phie = fly_vector.phi - y_oss[-1][9]
        Thetae = fly_vector.theta - y_oss[-1][10]
        Psie = fly_vector.psi - y_oss[-1][11]

        residui = [Xe, Ye, Ze, vXe, vYe, vZe, Pe, Qe, Re, Phie, Thetae, Psie]
        """
        Calcolo residui e assegnazione
        """

        #output_lines['00_ObservationFrequency'] = 0
        output_lines['01_Timestamp'] = fly_vector.timestamp
        output_lines['02_X'] = f"{Xe:.9f}"
        output_lines['03_Y'] = f"{Ye:.9f}"
        output_lines['04_Z'] = f"{Ze:.9f}"
        output_lines['05_vX'] = f"{vXe:.9f}"
        output_lines['06_vY'] = f"{vYe:.9f}"
        output_lines['07_vZ'] = f"{vZe:.9f}"
        output_lines['08_P'] = f"{Pe:.9f}"
        output_lines['09_Q'] = f"{Qe:.9f}"
        output_lines['10_R'] = f"{Re:.9f}"
        output_lines['11_Phi'] = f"{Phie:.9f}"
        output_lines['12_Theta'] = f"{Thetae:.9f}"
        output_lines['13_Psi'] = f"{Psie:.9f}"
        """
        Assegnazioni delle variabili residuo per la stampa.
        """

        fly_data.append(fly_vector)
        graph_vector.update(fly_vector, residui)
        """
        Aggiornamento vettori archivio.
        """

        try:  # freq max 111hz (sync ogni 9ms)
            """
            Try per l'interruzione manuale dello script.
            """
            ######################################################################################
            while True:
                """
                Osservazioni per stati successivi allo stato iniziale.
                Maggiori info sull'implementazione e l'idea di fondo sul pdf allegato.
                """
                fly_vector = getFlyVector(connection)
                
                #hz = 1000 / (fly_vector.timestamp - fly_data[-1].timestamp) #Calcolo HZ di connessione

                u_y = np.array([ #Generazione ingresso attuale
                    fly_vector.uf,
                    fly_vector.up,
                    fly_vector.uq,
                    fly_vector.ur,
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
                    fly_vector.psi
                ])

                u_y_old = np.array([ #Generazione ingresso precedente
                    fly_data[-1].uf,
                    fly_data[-1].up,
                    fly_data[-1].uq,
                    fly_data[-1].ur,
                    fly_data[-1].x,
                    fly_data[-1].y,
                    fly_data[-1].z,
                    fly_data[-1].vx,
                    fly_data[-1].vy,
                    fly_data[-1].vz,
                    fly_data[-1].p,
                    fly_data[-1].q,
                    fly_data[-1].r,
                    fly_data[-1].phi,
                    fly_data[-1].theta,
                    fly_data[-1].psi
                ])

                U=np.array([u_y_old, u_y]) #Generazione vettore di vettori di ingressi da passare all'elaborazione
                T=np.linspace(fly_data[-1].timestamp, fly_vector.timestamp, num=2) - fly_data[-1].timestamp #Generazione dei due istanti temporali da passare all'elaborazione
                
                y_oss, _, x_oss = signal.lsim(Obs, 
                                                U=U, 
                                                T=T, 
                                                X0=x_ossprec[-1])
                """
                Funzione lsim di calcolo dell'evoluzione del sistema lineare Osservatore.
                """

                Xe = fly_vector.x - y_oss[-1][0] #Calcolo residui
                Ye = fly_vector.y - y_oss[-1][1]
                Ze = fly_vector.z - y_oss[-1][2]
                vXe = fly_vector.vx - y_oss[-1][3]
                vYe = fly_vector.vy - y_oss[-1][4]
                vZe = fly_vector.vz - y_oss[-1][5]
                Pe = fly_vector.p - y_oss[-1][6]
                Qe = fly_vector.q - y_oss[-1][7]
                Re = fly_vector.r - y_oss[-1][8]
                Phie = fly_vector.phi - y_oss[-1][9]
                Thetae = fly_vector.theta - y_oss[-1][10]
                Psie = fly_vector.psi - y_oss[-1][11]
                
                residui = [Xe, Ye, Ze, vXe, vYe, vZe, Pe, Qe, Re, Phie, Thetae, Psie]
                """
                Calcolo residui e assegnazione.
                """

                #output_lines['00_ObservationFrequency'] = int(hz)
                output_lines['01_Timestamp'] = fly_vector.timestamp
                output_lines['02_X'] = f"{Xe:.9f}"
                output_lines['03_Y'] = f"{Ye:.9f}"
                output_lines['04_Z'] = f"{Ze:.9f}"
                output_lines['05_vX'] = f"{vXe:.9f}"
                output_lines['06_vY'] = f"{vYe:.9f}"
                output_lines['07_vZ'] = f"{vZe:.9f}"
                output_lines['08_P'] = f"{Pe:.9f}"
                output_lines['09_Q'] = f"{Qe:.9f}"
                output_lines['10_R'] = f"{Re:.9f}"
                output_lines['11_Phi'] = f"{Phie:.9f}"
                output_lines['12_Theta'] = f"{Thetae:.9f}"
                output_lines['13_Psi'] = f"{Psie:.9f}"

                x_ossprec=x_oss #Aggiornamento stato precedente

                fly_data.append(fly_vector)
                graph_vector.update(fly_vector, residui)
                """
                Aggiornamento vettori archivio.
                """
        
        except KeyboardInterrupt: #Interruzione script con KeyboardInterrupt per stampa risultati
            pass
            print('Script interrotto: stampa dei risultati in corso...')

        ###################################################################################### FINE WHILE OSSERVAZIONE

    printGraphs(graph_vector)
    """
    Stampa dei grafici.
    """
        
if __name__ == "__main__":
    main()
