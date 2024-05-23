global container_transfer

class container_transfer:
    _container = []
    
    @classmethod
    def get_container(cls):
        return cls._container
    
    @classmethod
    def set_container(cls, container):
        cls._container = container