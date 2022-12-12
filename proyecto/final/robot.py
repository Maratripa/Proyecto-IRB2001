from parametros import K2_D, K2_I, K1_D, K1_I

def sign(num):
    return 1 if num >= 0 else -1

class Robot:
    def __init__(self, data = {}):
        self.data = data

    def move_to_obj(self) -> tuple[float, float]:
        angle = self.data['a'] # type: float
        if abs(angle) > 90:
            m0 = K2_I * angle * sign(angle)
            m1 = K2_D * angle * sign(angle)

            return m0, m1
        else:
            m0 = K1_I * self.data['d1'] + K2_I * angle
            m1 = K1_D * self.data['d1'] + K2_D * angle

            return m0, m1