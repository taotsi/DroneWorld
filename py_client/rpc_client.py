import msgpackrpc

client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))
val = client.call("test_stixel")
print(val)
