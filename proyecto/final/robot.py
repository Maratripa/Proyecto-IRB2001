def sign(num):
    return 1 if num >= 0 else -1

class Robot:
    def __init__(self, data = {}):
        self.data = data
        self.state = "center"

    def move_to_obj(self) -> tuple[float, float]:
        angle = self.data['a'] # type: float
        dist_aa = self.data['d0']
        dist = self.data['d1']
        d_a1 = self.data['d3']
        a_a1 = self.data['a3']
        d_a2 = self.data['d4']
        a_a2 = self.data['a4']

        if self.state == "def_1":
            if d_a1 < 2.5 * dist_aa:
                m0 = 0.3 * angle
                m1 = -0.3 * angle
            else:
                if abs(a_a1) > 90:
                    m0 = 0.3 * a_a1 * sign(a_a1)
                    m1 = -0.3 * a_a1 * sign(a_a1)
                elif abs(a_a1) > 30:
                    m0 = 0.1 * d_a1 + 0.3 * a_a1 * sign(a_a1)
                    m1 = 0.1 * d_a1 - 0.3 * a_a1 * sign(a_a1)
                elif abs(a_a1) > 15:
                    m0 = 0.15 * d_a1 + 0.2 * a_a1 * sign(a_a1)
                    m1 = 0.15 * d_a1 - 0.2 * a_a1 * sign(a_a1)
                else:
                    m0 = 0.2 * d_a1 + 0.2 * a_a1 * sign(a_a1)
                    m1 = 0.2 * d_a1 - 0.2 * a_a1 * sign(a_a1)
        elif self.state == "def_3":
            if dist < 6 * dist_aa:
                if abs(angle) > 90:
                    m0 = 0.3 * angle
                    m1 = -0.3 * angle
                elif abs(angle) > 30:
                    m0 = 0.1 * dist + 0.3 * angle
                    m1 = 0.1 * dist - 0.3 * angle
                elif abs(angle) > 15:
                    m0 = 0.15 * dist + 0.2 * angle
                    m1 = 0.15 * dist - 0.2 * angle
                else:
                    m0 = 0.2 * dist + 0.2 * angle
                    m1 = 0.2 * dist - 0.2 * angle
            else:
                m0 = 0.3 * angle
                m1 = -0.3 * angle
        else:
            if abs(angle) > 90:
                m0 = 0.3 * angle
                m1 = -0.3 * angle
            elif abs(angle) > 30:
                m0 = 0.1 * dist + 0.3 * angle
                m1 = 0.1 * dist - 0.3 * angle
            elif abs(angle) > 15:
                m0 = 0.15 * dist + 0.2 * angle
                m1 = 0.15 * dist - 0.2 * angle
            else:
                if dist < 2.5 * dist_aa and self.state == "no tocar":
                    m0 = 0
                    m1 = 0
                elif dist < 2.5 * dist_aa and self.state == "line":
                    m0 = 50
                    m1 = 50
                else:
                    m0 = 0.2 * dist + 0.2 * angle
                    m1 = 0.2 * dist - 0.2 * angle
        
        print(f"Angulo: {angle}\t|\tD1: {dist}\t|\tm0: {m0}\t|\tm1: {m1}")
        
        return m0, m1