global order_data_transfer

class order_data_transfer:
    _order_data = []
    
    @classmethod
    def get_order_data(cls):
        return cls._order_data
    
    @classmethod
    def set_order_data(cls, order_data):
        cls._order_data = order_data