import numpy as np
import matplotlib.pyplot as plt

# Load the data
dataRec = np.loadtxt("datalog.log", delimiter=",")

# Extract relevant data columns
simTime = dataRec[:, 0:1]
motor_pos_des = dataRec[:, 1:32]
motor_pos_cur = dataRec[:, 32:63]
motor_vel_cur = dataRec[:, 63:94]
motor_tor_des = dataRec[:, 94:125]
motor_tor_out = dataRec[:, 125:156]
rpyVal = dataRec[:, 156:159]
gpsVal = dataRec[:, 159:162]
fe_l_pos_L_des = dataRec[:, 162:165]
fe_r_pos_L_des = dataRec[:, 165:168]
fe_l_pos_W = dataRec[:, 168:171]
fe_r_pos_W = dataRec[:, 171:174]
Ufe = dataRec[:, 174:186]

# Get data size and range
rowt, colt = simTime.shape
ranget = np.arange(rowt)

# Plot "motor pos"
plt.figure("motor pos")
for i in range(3):
    plt.subplot(3, 6, i + 1)
    plt.plot(
        simTime[ranget, 0],
        motor_pos_cur[ranget, i + 16] * 180 / np.pi,
        "b-",
        label="Current",
    )
    plt.plot(
        simTime[ranget, 0],
        motor_pos_des[ranget, i + 16] * 180 / np.pi,
        "r-",
        label="Desired",
    )
    plt.legend()

for i in range(12):
    plt.subplot(3, 6, i + 7)
    plt.plot(
        simTime[ranget, 0],
        motor_pos_cur[ranget, i + 19] * 180 / np.pi,
        "b-",
        label="Current",
    )
    plt.plot(
        simTime[ranget, 0],
        motor_pos_des[ranget, i + 19] * 180 / np.pi,
        "r-",
        label="Desired",
    )
    plt.legend()

# Plot "motor tor out"
plt.figure("motor tor out")
for i in range(3):
    plt.subplot(3, 6, i + 1)
    plt.plot(simTime[ranget, 0], motor_tor_out[ranget, i + 16], "b-", label="Output")
    plt.plot(simTime[ranget, 0], motor_tor_des[ranget, i + 16], "r-", label="Desired")
    plt.legend()

for i in range(12):
    plt.subplot(3, 6, i + 7)
    plt.plot(simTime[ranget, 0], motor_tor_out[ranget, i + 19], "b-", label="Output")
    plt.plot(simTime[ranget, 0], motor_tor_des[ranget, i + 19], "r-", label="Desired")
    plt.legend()

# Plot "Ufe"
plt.figure("Ufe")
for i in range(12):
    plt.subplot(4, 3, i + 1)
    plt.plot(simTime[ranget, 0], Ufe[ranget, i])
    plt.grid()

# Plot "fe"
plt.figure("fe")
for i in range(3):
    plt.subplot(2, 3, i + 1)
    plt.plot(simTime[ranget, 0], fe_l_pos_W[ranget, i])
    plt.grid()

    plt.subplot(2, 3, i + 4)
    plt.plot(simTime[ranget, 0], fe_r_pos_W[ranget, i])
    plt.grid()

# Show all plots
plt.show()
