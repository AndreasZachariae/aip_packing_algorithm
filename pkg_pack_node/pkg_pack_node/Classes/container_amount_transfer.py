global container_amount_transfer

class container_amount_transfer:
    _container_amount = 0
    
    @classmethod
    def get_container_amount(cls):
        return cls._container_amount
    
    @classmethod
    def set_container_amount(cls, container_amount):
        cls._container_amount = container_amount