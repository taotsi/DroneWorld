import msgpackrpc


class DataClient:
    def __init__(self):
        self.client_ = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))

    def test(self):
        return self.client_.call("test")

    def GetKde(self):
        return self.client_.call("GetKde")

    def GetPillarFrame(self):
        return self.client_.call("GetPillarFrame")

    def GetPillarCluster(self):
        return self.client_.call("GetPillarCluster")

    def GetKdeZ1(self):
        return self.client_.call("GetKdeZ1")

    def GetKdeZ2(self):
        return self.client_.call("GetKdeZ2")
