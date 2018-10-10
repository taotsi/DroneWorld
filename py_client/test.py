from rpc_client import DataClient
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def DrawKde(kde):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_len = len(kde)
    y_len = len(kde[0])
    x_list = np.arange(0, x_len, 1)
    y_list = np.arange(0, y_len, 1)
    x, y = np.meshgrid(x_list, y_list, indexing="ij")
    z = np.array(kde)
    surf = ax.plot_surface(x, y, z, alpha=0.7)
    plt.show()


def DrawPillarFrame(pillars):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for pl in pillars:
        # print(pl)
        ax.plot([pl[0], pl[0]], [pl[1], pl[1]], [pl[2], pl[3]])
    ax.axis("equal")
    ax.set_title("pillars in one frame")
    plt.show()


def DrawPillarCluster(clusters):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    n_cluster = len(clusters)
    for i, cl in enumerate(clusters):
        color = (i / n_cluster, 1 - i / n_cluster, 0)
        for pl in cl:
            ax.plot([pl[0], pl[0]], [pl[1], pl[1]],
                    [pl[2], pl[3]], color=color)
    ax.axis("equal")
    ax.set_title("pillar cluster horizontally")
    plt.show()


if __name__ == "__main__":
    client = DataClient()

    # kde = client.GetKde()
    # DrawKde(kde)
    #pillars = client.GetPillarFrame()
    # print(len(pillars))
    # DrawPillarFrame(pillars)
    # clusters_h = client.GetPillarClusterHorizon()
    clusters = client.GetPillarCluster()
    DrawPillarCluster(clusters)
