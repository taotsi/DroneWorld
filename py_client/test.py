from rpc_client import DataClient
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random


def DrawDisparityFrame(frame):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_len = len(frame)
    y_len = len(frame[0])
    x_array = np.arange(0, x_len, 1)
    y_array = np.arange(0, y_len, 1)
    x, y = np.meshgrid(x_array, y_array, indexing='ij')
    z = np.array(frame)
    ax.plot_surface(x, y, z, alpha=0.7)
    plt.show()


def DrawKde(kde):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_len = len(kde)
    y_len = len(kde[0])
    x_list = np.arange(0, x_len, 1)
    y_list = np.arange(0, y_len, 1)
    x, y = np.meshgrid(x_list, y_list, indexing="ij")
    z = np.array(kde)
    ax.plot_surface(x, y, z, alpha=0.7)
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
    for i, cl in enumerate(clusters):
        color = (random.uniform(0, 1), random.uniform(
            0, 1), random.uniform(0, 1))
        for pl in cl:
            ax.plot([pl[0], pl[0]], [pl[1], pl[1]],
                    [pl[2], pl[3]], color=color)
    ax.axis("equal")
    ax.set_title("pillar cluster")
    plt.show()


def DrawPlanes(planes):
    fig = plt.figure()
    ax = Axes3D(fig)
    for plane in planes:
        x = [plane[0][0], plane[1][0], plane[1][0], plane[0][0]]
        y = [plane[0][1], plane[1][1], plane[1][1], plane[0][1]]
        z = [plane[0][2], plane[0][2], plane[1][2], plane[1][2]]
        verts = [list(zip(x, y, z))]
        ax.add_collection3d(Poly3DCollection(verts), zs='z')
    plt.show()


if __name__ == "__main__":
    client = DataClient()

    frame = client.GetDisparityFrame()
    DrawDisparityFrame(frame)
    kde = client.GetKde()
    DrawKde(kde)
    pillars = client.GetPillarFrame()
    DrawPillarFrame(pillars)
    clusters = client.GetPillarCluster()
    DrawPillarCluster(clusters)
    filtered_clusters = client.GetFilteredCluster()
    print(len(filtered_clusters))
    DrawPillarCluster(filtered_clusters)
    planes = client.GetPlanes()
    DrawPlanes(planes)
