global items_transfer

class items_transfer:
    _items_data = []
    
    @classmethod
    def get_items(cls):
        return cls._items_data
    
    @classmethod
    def set_items(cls, items_data):
        cls._items_data = items_data