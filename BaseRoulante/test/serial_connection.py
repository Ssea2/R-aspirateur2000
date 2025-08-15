import serial 
import matplotlib.pyplot as plt
import numpy as np


arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600)


def read_data():
    data = arduino.readline()
    return data

def write_data(x):
    arduino.write(bytes(x,  'utf-8'))

Ls = []
Rs = []

while True:
    value  = read_data()
    c,x,y= str(value)[2:-3].split(",")
    print(" \n\n")
    print(value)
    print(c)
    print("L",x)
    print("R",y)
    print(" \n\n")
    Ls.append(x)
    Rs.append(y)
    if float(c)>=127.0:
        break



with open("encodeur_value.txt","w+") as file:
    data = str(Ls)+";"+str(Rs)
    file.write(data)


with open("encodeur_value.txt", "r") as file:
    data = file.read()


L, R = data.split(";")
L = [np.float64(i.replace("'","")) for i in L[1:-1].split(",")]
R = [np.float64(i.replace("'","")) for i in R[1:-1].split(",")]


x=[20+j for j in range(len(R))]
"""plt.figure()
plt.plot( x,L, label="L")
plt.plot( x,R,label="R")
plt.legend()
plt.show()"""

coeffs = np.polyfit(R, L, 1)  # degré 1 = linéaire
a, b = coeffs
print(f"Relation estimée : R ≈ {a:.4f} * L + {b:.4f}")

# Vérification visuelle :
print(type(a), type(b), set([type(i) for i in L]))
R_fit = [a * xi + b for xi in R]
plt.plot(x, L, label='L')
plt.plot(x, R, label='R')
plt.plot(x, R_fit, '--', label='Fit R(L)')
plt.legend()
plt.show()