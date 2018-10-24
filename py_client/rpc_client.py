import msgpackrpc


class DataClient:
    def __init__(self):
        self.client_ = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 8080))

    def test(self):
        print(self.client_.call("test"))

    def GetDisparityFrame(self):
        return self.client_.call("GetDisparityFrame")

    def GetKde(self):
        return self.client_.call("GetKde")

    def GetPillarFrame(self):
        return self.client_.call("GetPillarFrame")

    def GetPillarCluster(self):
        return self.client_.call("GetPillarCluster")

    def GetFilteredCluster(self):
        return self.client_.call("GetFilteredCluster")
