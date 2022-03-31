from pymavlink import mavutil
import numpy as np
import control
import control.matlab as signal
from reprint import output
from time import sleep
import math

from sqlite3 import Timestamp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
        self.up = uq
        self.uq = up
        self.ur = ur

class GraphVector:
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



def initializeMockConnection():
    inputfile = open("test_input", mode='r')
    inputfile.readline()
    return inputfile

def getMockFlyVector(connection):
    line = connection.readline()
    if line:
        timestamp = int(line.split(',')[0])
        x = float(line.split(',')[1])
        y = float(line.split(',')[2])
        z = -float(line.split(',')[3])
        vx = float(line.split(',')[4])
        vy = float(line.split(',')[5])
        vz = -float(line.split(',')[6])
        p = float(line.split(',')[7])
        q = float(line.split(',')[8])
        r = float(line.split(',')[9])
        phi = float(line.split(',')[10])
        theta = float(line.split(',')[11])
        psi = float(line.split(',')[12])
        uf = float(line.split(',')[13])
        up = float(line.split(',')[14])
        uq = float(line.split(',')[15])
        ur = float(line.split(',')[16])

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
    else:
        return 0

def printGraphs(graph_vector):
        
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

    with open("test_input", mode='r') as connection:
        connection.readline()

        fly_data = [FlyVector()]
        graph_vector = GraphVector()

        A0 = np.diag([-4, -4.8, -4.2, -4.3, -4.5, -4.9, -5, -4.1, -4.4, -4.75, -4.25, -4.55]) # Deriva da teoria Osservatore
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
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 4.55]]) # Deriva da teoria Osservatore
        C0 = np.identity(12)
        D0 = np.zeros((12, 16))

        Obs = control.ss(A0, B0, C0, D0)
        # _____________________________________________ FINE DEFINIZIONE SISTEMA E MATRICI

        with output(output_type='dict') as output_lines:

            fly_vector = getMockFlyVector(connection)

            if fly_vector:
                u_y = np.array([
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

                y_oss, _, x_ossprec = signal.lsim(Obs, U=np.array([np.zeros(16), u_y]), T=np.linspace(0, fly_vector.timestamp, num=2, endpoint=True))

                output_lines['Timestamp'] = fly_vector.timestamp
                output_lines['X'] = y_oss[-1][0]
                output_lines['Y'] = y_oss[-1][1]
                output_lines['Z'] = y_oss[-1][2]
                output_lines['vX'] = y_oss[-1][3]
                output_lines['vY'] = y_oss[-1][4]
                output_lines['vZ'] = y_oss[-1][5]
                output_lines['P'] = y_oss[-1][6]
                output_lines['Q'] = y_oss[-1][7]
                output_lines['R'] = y_oss[-1][8]
                output_lines['Phi'] = y_oss[-1][9]
                output_lines['Theta'] = y_oss[-1][10]
                output_lines['Psi'] = y_oss[-1][11]

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

                fly_data.append(fly_vector)
                graph_vector.update(fly_vector, residui)


            else:
                connection = False

            try:
                while connection:  # freq max 111hz (sync ogni 9ms)

                    #fly_vector = getFlyVector(connection, fly_data)
                    fly_vector = getMockFlyVector(connection)
                    if fly_vector:
                        u_y = np.array([
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

                        u_y_old = np.array([
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

                        #ELABORAZIONE
                        U=np.array([u_y_old, u_y])
                        T=np.linspace(fly_data[-1].timestamp, fly_vector.timestamp, num=2) - fly_data[-1].timestamp
                        
                        y_oss, _, x_oss = signal.lsim(Obs, 
                                                        U=U, 
                                                        T=T, 
                                                        X0=x_ossprec[-1])

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

                        output_lines['###PRESS CTRL-C TO INTERRUPT'] = ''
                        output_lines['Timestamp'] = fly_vector.timestamp
                        output_lines['X'] = f"{Xe:.9f}"
                        output_lines['Y'] = f"{Ye:.9f}"
                        output_lines['Z'] = f"{Ze:.9f}"
                        output_lines['vX'] = f"{vXe:.9f}"
                        output_lines['vY'] = f"{vYe:.9f}"
                        output_lines['vZ'] = f"{vZe:.9f}"
                        output_lines['P'] = f"{Pe:.9f}"
                        output_lines['Q'] = f"{Qe:.9f}"
                        output_lines['R'] = f"{Re:.9f}"
                        output_lines['Phi'] = f"{Phie:.9f}"
                        output_lines['Theta'] = f"{Thetae:.9f}"
                        output_lines['Psi'] = f"{Psie:.9f}"

                        x_ossprec=x_oss

                        fly_data.append(fly_vector)
                        graph_vector.update(fly_vector, residui)

                    else:
                        connection = False
                        print('File terminato!')
            
            except KeyboardInterrupt:
                    pass    
                    print('Interrupted')

        # ____________________________________________ FINE WHILE
    
    printGraphs(graph_vector)
        


        
        
if __name__ == "__main__":
    main()
