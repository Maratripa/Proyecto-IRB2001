import numpy as np

def sign(num):
    return 1 if num >= 0 else -1

def get_proyection(arco, pelota, pos_robot, angulo_deg, distancia):
    angulo = np.deg2rad(angulo_deg)

    vec_arco_pelota = np.array(arco) - np.array(pelota)
    angulo_arco_pelota = np.arctan2(vec_arco_pelota[1], vec_arco_pelota[0])

    x1 = pelota[0] + distancia * np.cos(angulo_arco_pelota + angulo)
    y1 = pelota[1] + distancia * np.sin(angulo_arco_pelota + angulo)

    x2 = pelota[0] + distancia * np.cos(angulo_arco_pelota - angulo)
    y2 = pelota[1] + distancia * np.sin(angulo_arco_pelota - angulo)

    pos1 = np.array((x1, y1))
    pos2 = np.array((x2, y2))

    dist1 = np.linalg.norm(np.array(pos_robot) - pos1)
    dist2 = np.linalg.norm(np.array(pos_robot) - pos2)

    if dist1 < dist2:
        return dist1
    else:
        return dist2

class Robot:
    def __init__(self, data = {}):
        self.data = data
        self.state = "stop"

    def move_to_obj(self) -> tuple[float, float]:
        angle = self.data['a'] # type: float
        dist_aa = self.data['d0']
        dist = self.data['d1']
        d_a1 = self.data['d3']
        a_a1 = self.data['a3']
        d_a2 = self.data['d4']
        a_a2 = self.data['a4']
        do = self.data['do']
        ao = self.data['ao']

        if self.state == "stop":
            m0, m1 = 0, 0

        elif self.state == "def_1":
            if d_a1 < 2.5 * dist_aa:
                m0 = 0.5 * angle
                m1 = -0.5 * angle
            else:
                if abs(a_a1) > 90:
                    m0 = 0.3 * a_a1
                    m1 = -0.3 * a_a1
                elif abs(a_a1) > 30:
                    m0 = 0.1 * d_a1 + 0.3 * a_a1
                    m1 = 0.1 * d_a1 - 0.3 * a_a1
                elif abs(a_a1) > 15:
                    m0 = 0.15 * d_a1 + 0.2 * a_a1
                    m1 = 0.15 * d_a1 - 0.2 * a_a1
                else:
                    m0 = 0.25 * d_a1 + 0.2 * a_a1
                    m1 = 0.25 * d_a1 - 0.2 * a_a1
        
        elif self.state == "def_2":
            if dist > 2 * dist_aa:
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
                    m0 = 0.25 * dist + 0.2 * angle
                    m1 = 0.25 * dist - 0.2 * angle
            else:
                m0 = 0.5 * angle
                m1 = -0.5 * angle
            
        elif self.state == "def_3":
            if dist < 10 * dist_aa:
                if abs(angle) > 90:
                    m0 = 1 * angle
                    m1 = -1 * angle
                elif abs(angle) > 30:
                    m0 = 0.5 * dist + 1 * angle
                    m1 = 0.5 * dist - 1 * angle
                elif abs(angle) > 15:
                    m0 = 0.6 * dist + 1 * angle
                    m1 = 0.6 * dist - 1 * angle
                else:
                    m0 = 0.7 * dist + 1 * angle
                    m1 = 0.7 * dist - 1 * angle
            else:
                m0 = 1 * angle
                m1 = -1 * angle
        
        elif self.state == "atk_1":
            if dist < 2.3 * dist_aa:
                if abs(a_a2) > 90:
                    m0 = 0.2 * a_a2
                    m1 = -0.2 * a_a2
                elif abs(a_a2) > 30:
                    m0 = 0.1 * d_a2 + 0.3 * a_a2
                    m1 = 0.1 * d_a2 - 0.3 * a_a2
                elif abs(a_a2) > 15:
                    m0 = 0.15 * d_a2 + 0.2 * a_a2
                    m1 = 0.15 * d_a2 - 0.2 * a_a2
                else:
                    m0 = 0.2 * d_a2 + 0.2 * a_a2
                    m1 = 0.2 * d_a2 - 0.2 * a_a2
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
                    m0 = 0.2 * dist + 0.2 * angle
                    m1 = 0.2 * dist - 0.2 * angle
        
        elif self.state == "atk_2":
            if do > 2 * dist_aa:
                if abs(ao) > 90:
                    m0 = 0.3 * ao
                    m1 = -0.3 * ao
                elif abs(ao) > 30:
                    m0 = 0.1 * do + 0.3 * ao
                    m1 = 0.1 * do - 0.3 * ao
                elif abs(ao) > 15:
                    m0 = 0.15 * do + 0.2 * ao
                    m1 = 0.15 * do - 0.2 * ao
                else:
                    m0 = 0.25 * do + 0.2 * ao
                    m1 = 0.25 * do - 0.2 * ao
            else:
                if abs(angle) < 5:
                    if dist < 2.4 * dist_aa:
                        m0, m1 = 30, 30
                        self.state = "stop"
                    else:
                        m0 = 60 + 0.1 * angle
                        m1 = 60 - 0.1 * angle
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
            
            if self.state == "center":
                if dist < 2.3 * dist_aa:
                    m0, m1 = 0, 0
        
        print(f"Angulo: {angle}\t|\tD1: {dist}\t|\tm0: {m0}\t|\tm1: {m1}\t|\tState: {self.state}")
        
        if max(m0, m1) > 100:
            div = max(m0, m1) / 100
        else:
            div = 1
        
        return m0 / div, m1 / div