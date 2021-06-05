

class Controller:

    def __init__(self):
        # P I D parameters
        self.KY_p = 1
        self.KX_p = 1

        # Reference paramers
        self.r_x = 320
        self.r_y = 256

        # previous variables for I and D?
        # x-1 x-2...

    def get_input(self,measure_x,measure_y):
        u_yaw = -self.KX_p * (self.r_x - measure_x)
        u_pitch = -self.KY_p * (self.r_y - measure_y)
        return u_yaw, u_pitch


if __name__=="__main__":
    pid = Controller()
    u_yaw, u_pitch = pid.get_input(5,10)
    print(u_pitch)
        

