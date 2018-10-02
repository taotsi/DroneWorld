import msgpackrpc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))

pillars = client.call("GetPillarFrame")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(len(pillars)):
    ax.plot([pillars[i][0], pillars[i][0]],
            [pillars[i][1], pillars[i][1]],
            [pillars[i][2], pillars[i][3]])

plt.show()
