import msgpackrpc
import matplotlib.pyplot as plt

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))

pillars = client.call("GetPillarFrame")
print(pillars)

"""
kde = client.call("GetKde")
plt.plot(kde)
plt.show()
"""
