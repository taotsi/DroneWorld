from rpc_client import DataClient
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def DrawPillarCluster(clusters):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for cl in clusters:
        for pl in cl:
            ax.plot([pl[0], pl[0]], [pl[1], pl[1]], [pl[2], pl[3]])
    plt.show()


if __name__ == "__main__":
    client = DataClient()
    clusters = client.GetPillarClusterHorizon()
    DrawPillarCluster(clusters)
