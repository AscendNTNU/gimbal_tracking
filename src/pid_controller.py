

class Controller:

    def __init__(self,K_p=1):
        # P I D parameters
        self.K_p = K_p

        # TODO Add I and D support
        # previous variables for I and D?
        # x-1 x-2...

    def get_input(self,error_x,error_y):
        u_yaw = -self.K_p * error_x
        u_pitch = -self.K_p * error_y
        return u_yaw, u_pitch


if __name__=="__main__":
    pid = Controller()
    u_yaw, u_pitch = pid.get_input(5,10)
    print(u_pitch)
        

