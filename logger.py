class Logger(object):
    def __init__(self):
        self.logs = {}
    def add(self, topic, data):
        if topic not in self.logs:
            self.logs[topic] = [data]
        else:
            self.logs[topic].append(data)
    def get_log(self, topic):
        if topic not in self.logs:
            print("No data in " + topic)
            return []
        else:
            return self.logs[topic]
    def clear(self):
        del self.logs
        self.logs = {}