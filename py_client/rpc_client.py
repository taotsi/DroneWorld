import msgpackrpc
import matplotlib.pyplot as plt

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))
stx = client.call("GetStixel")
plt.plot(stx)
plt.show()
