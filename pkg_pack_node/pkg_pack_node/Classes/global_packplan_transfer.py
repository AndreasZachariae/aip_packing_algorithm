global packplan_temp

class Packplan_temp:
    _packplan_temp = []
    
    @classmethod
    def get_packplan_temp(cls):
        return cls._packplan_temp
    
    @classmethod
    def set_packplan_temp(cls, packplan_temp):
        cls._packplan_temp = packplan_temp