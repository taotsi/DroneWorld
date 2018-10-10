import msgpackrpc


class DataClient:
    def __init__(self):
        self.client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))

    def test(self):
        return self.client.call("test")

    def GetKde(self):
        return self.client.call("GetKde")

    def GetPillarFrame(self):
        return self.client.call("GetPillarFrame")

    def GetPillarClusterHorizon(self):
        return self.client.call("GetPillarClusterHorizon")

    def GetPillarCluster(self):
        return self.client.call("GetPillarCluster")


"""

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(len(pillars)):
    ax.plot([pillars[i][0], pillars[i][0]],
            [pillars[i][1], pillars[i][1]],
            [-pillars[i][2], -pillars[i][3]])
plt.show()
"""
