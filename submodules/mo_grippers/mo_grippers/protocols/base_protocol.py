class BaseProtocol:
    def send(self, *args, **kwargs):
        raise NotImplementedError("Must be implemented by subclasses")

    def receive(self, *args, **kwargs):
        raise NotImplementedError("Must be implemented by subclasses")

    def close(self):
        raise NotImplementedError("Must be implemented by subclasses")
