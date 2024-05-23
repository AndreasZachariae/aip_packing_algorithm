global Order_class
class Order_class:
    _topic_data = ""

    @classmethod
    def get_topic_data(cls):
        return cls._topic_data
    
    @classmethod
    def set_topic_data(cls,topic_data):
        cls._topic_data = topic_data