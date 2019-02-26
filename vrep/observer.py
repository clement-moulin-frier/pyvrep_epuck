from collections import defaultdict
from queue import Queue




class Observable(object):
    def __init__(self):
        self.subscribers = defaultdict(list)

    def subscribe(self, topic, subscriber):
        self.subscribers[topic].append(subscriber)

    def unsubscribe(self, topic, subscriber):
        self.subscribers[topic].remove(subscriber)

    def emit(self, topic, message):

        for subscriber in self.subscribers[topic]:
            subscriber._wrapped_handle_notification(topic, message)


class Observer(object):
    def __init__(self):
        self.notifications = Queue()

    def _wrapped_handle_notification(self, topic, message):
        self.notifications.put((topic, message))
        self.handle_notification(topic, message)

    def handle_notification(self, topic, message):
        pass

    def poll_notification(self):
        return self.notifications.get()


if __name__ == '__main__':
    class ObjectTracker(Observable):
        pass

    class EventManager(Observer):
        def handle_notification(self, topic, message):
            print('new event on topic "{}": "{}"'.format(topic, message))

    object_tracker = ObjectTracker()
    event_manager = EventManager()

    object_tracker.subscribe('appear', event_manager)
    object_tracker.emit('appear', 'gripper')

    topic, msg = event_manager.poll_notification()