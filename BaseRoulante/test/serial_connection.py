import serial 
import matplotlib.pyplot as plt
import numpy as np
import time 

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
    print(value)
    c,L,R= str(value)[2:-3].split(",")
    print(" \n\n")
    print(value)
    print("compteur",c)
    print("L",L)
    print("R",R)
    print(" \n\n")
    Ls.append(L)
    Rs.append(R)
    print(c)
    if int(c)>=2000:
        break


with open("encodeur_value_mps6.txt","w+") as file:
    data = str(Ls)+";"+str(Rs)
    file.write(data)


with open("encodeur_value_mps6.txt", "r") as file:
    data = file.read()


L, R = data.split(";")

L = [np.float64(i.replace("'","")) for i in L[1:-1].split(",")]
R = [-1*np.float64(i.replace("\\\\r","").replace("'","")) for i in R[1:-1].split(",")]
print( np.mean(L), np.mean(R))

x=[j for j in range(len(R))]
plt.figure()
plt.plot( x,L, label="L")
plt.scatter( x,L)
plt.plot( x,R,label="R")
plt.scatter( x,R)
plt.legend()
plt.show()


# #coeffs = np.polyfit(R, L, 1)  # degré 1 = linéaire
# a, b = coeffs
# print(f"Relation estimée : R ≈ {a:.4f} * L + {b:.4f}")

# # Vérification visuelle :
# print(type(a), type(b), set([type(i) for i in L]))
# R_fit = [a * xi + b for xi in R]
# plt.plot(x, L, label='L')
# plt.plot(x, R, label='R')
# plt.plot(x, R_fit, '--', label='Fit R(L)')
# plt.legend()
# plt.show()""""""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
import plotly.express as px 

# Exemple de données
#x = np.array([20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34])
y1 = L
y2 = R

# Générer plus de points pour lisser
"""x_new = np.linspace(min(x), max(x), 300)

# Créer spline pour chaque série
spl1 = make_interp_spline(x, y1, k=3)  # k=3 -> spline cubique
spl2 = make_interp_spline(x, y2, k=3)

y1_smooth = spl1(x_new)
y2_smooth = spl2(x_new)

# Tracer
plt.plot(x_new, y1_smooth, label="L (lissé)")
plt.plot(x_new, y2_smooth, label="R (lissé)")
plt.scatter(x, y1, c='blue', s=20)  # points bruts
plt.scatter(x, y2, c='orange', s=20)
plt.legend()
plt.show()"""

# def moving_average(data, window_size=3):
#     return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# # Exemple avec y1
# y1_ma = moving_average(y1, window_size=3)
# y2_ma = moving_average(y2, window_size=3)

# """plt.plot(x[len(x)-len(y1_ma):], y1_ma, label="L (moyenne mobile)")
# plt.plot(x[len(x)-len(y2_ma):], y2_ma, label="R (moyenne mobile)")
# plt.scatter(x[len(x)-len(y1_ma):], y1_ma, c='blue', s=20)
# plt.scatter(x[len(x)-len(y2_ma):], y2_ma, c='orange', s=20)
# plt.legend()
# plt.show()"""

# import pandas as pd
# import plotly.express as px

# # Exemple avec tes données
# # x, y1_ma, y2_ma doivent déjà exister

# df = pd.DataFrame({
#     "x": x[len(x)-len(y1_ma):],
#     "L (moyenne mobile)": y1_ma,
#     "R (moyenne mobile)": y2_ma
# })

# # Transformer en format long pour px.line et px.scatter
# df_long = df.melt(id_vars="x", var_name="Série", value_name="Valeur")

# # Tracer la ligne
# fig = px.line(df_long, x="x", y="Valeur", color="Série")

# # Ajouter les points
# fig.add_scatter(x=df["x"], y=df["L (moyenne mobile)"], 
#                 mode="markers", marker=dict(color="blue", size=6), 
#                 name="L (points)")

# fig.add_scatter(x=df["x"], y=df["R (moyenne mobile)"], 
#                 mode="markers", marker=dict(color="orange", size=6), 
#                 name="R (points)")


# x_full = np.linspace(min(x), max(x), 500)  # 500 points réguliers

# # Interpolation des courbes
# y1_interp = np.interp(x_full, x[len(x)-len(y1_ma):], y1_ma)
# y2_interp = np.interp(x_full, x[len(x)-len(y2_ma):], y2_ma)

# df_interp = pd.DataFrame({
#     "x": x_full,
#     "motor": (y1_interp+y2_interp)/2
# })

# # Format long
# df_long = df_interp.melt(id_vars="x", var_name="Série", value_name="Valeur")

# # Courbe avec points + hover sur chaque valeur interpolée
# fig = px.line(df_long, x="x", y="Valeur", color="Série", markers=False)

# fig.update_traces(hovertemplate="x=%{x}<br>Valeur=%{y}<extra>%{legendgroup}</extra>")

# fig.show()