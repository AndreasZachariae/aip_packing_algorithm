global request_transfer

class request_transfer:
    _request_data = []
    
    @classmethod
    def get_request(cls):
        return cls._request_data
    
    @classmethod
    def set_request(cls, request_data):
        cls._request_data = request_data