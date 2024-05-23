global packplan

class Packplan:
    _packplan = []
    
    @classmethod
    def get_packplan(cls):
        return cls._packplan
    
    @classmethod
    def set_packplan(cls, packplan):
        cls._packplan = packplan