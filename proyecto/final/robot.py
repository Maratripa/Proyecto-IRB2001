def sign(num):
    return 1 if num >= 0 else -1

class Robot:
    def __init__(self, data = {}):
        self.data = data

    def move_to_obj(self) -> tuple[float, float]:
        angle = self.data['a'] # type: float
        dist = self.data['d1']
        if abs(angle) > 90:
            m0 = 0.3 * angle * sign(angle)
            m1 = -0.3 * angle * sign(angle)
        elif abs(angle) > 30:
            m0 = 0.1 * dist + 0.3 * angle * sign(angle)
            m1 = 0.1 * dist - 0.3 * angle * sign(angle)
        elif abs(angle) > 15:
            m0 = 0.15 * dist + 0.2 * angle * sign(angle)
            m1 = 0.15 * dist - 0.2 * angle * sign(angle)
        else:
            m0 = 0.2 * dist + 0.2 * angle * sign(angle)
            m1 = 0.2 * dist - 0.2 * angle * sign(angle)
        
        print(f"Angulo: {angle}\t|\tD1: {dist}\t|\tm0: {m0}\t|\tm1: {m1}")
        
        return m0, m1