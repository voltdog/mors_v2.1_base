import hid
import time

RX = 0
RY = 1
LX = 2
LY = 3
R1 = 5
R2 = 4
L2 = 6
L1 = 7

VENDOR_ID = 0x0483
DEVICE_ID = 0x5710

CONNECTED = 1
RECEIVER_ERROR = 2
JOYSTICK_ERROR = 3
ACCESS_ERROR = 4

class Joystick:
    def __init__(self) -> None:
        self.joy_value = [0]*8
        self.__status = 0
        self.__pause = False
        self.connect()


    def connect(self):
        self.device = hid.device()
        try:
            self.device.open(VENDOR_ID, DEVICE_ID)
        # except OSError:
        #     self.status = ACCESS_ERROR
        except:
            self.__status = RECEIVER_ERROR
            return
        self.__status = CONNECTED
        

    def read_data(self):
        if self.__status != RECEIVER_ERROR:
            try:
                data = self.device.read(16, 100)
            except:
                self.__status = JOYSTICK_ERROR
                return -1
            # print((data[10] | (data[11] << 8))-557)

            if len(data) == 16:
                self.joy_value[RX] = (data[0] | (data[1] << 8))/16285 - 1.0
                self.joy_value[RY] = -(data[2] | (data[3] << 8))/16285 + 1.0
                self.joy_value[LY] = (data[4] | (data[5] << 8))/16285 - 1.0
                self.joy_value[LX] = (data[6] | (data[7] << 8))/16285 - 1.0
                self.joy_value[R2] = int(round((data[8] | (data[9] << 8))/16285)) # правый курок
                self.joy_value[R1] = (data[10] | (data[11] << 8)) # R1 button
                self.joy_value[L2] = int(round((data[12] | (data[13] << 8))/16285)) # left курок
                self.joy_value[L1] = (data[14] | (data[15] << 8))#/16285 - 1.0
                # print(self.joy_value[L2])
                if self.joy_value[R1] > 1000:
                    self.joy_value[R1] = True
                else:
                    self.joy_value[R1] = False
                # print(self.joy_value[R1])
                if self.joy_value[3:6] == [-1.0, 1, True]:
                    self.__status = JOYSTICK_ERROR
                    return -1
                else:
                    self.__status = CONNECTED
                return 1
            else:
                self.__status = JOYSTICK_ERROR
                return -1
    
    def loop_read(self):
        while True:
            if self.__pause == False:
                if self.read_data() == -1:
                    self.__pause = True
                    # self.joy_value == [0, 0, 0, 0, 1, False]

    def pause_loop(self):
        self.__pause = True

    def play_loop(self):
        self.__pause = False

    def get_joy_data(self):
        return self.joy_value


    def get_joy_status(self):
        return self.__status
    


if __name__ == "__main__":
    joy = Joystick()

    while(1):
        status = joy.get_joy_status()

        if status == CONNECTED:
            if joy.read_data() == 1:
                lst = joy.get_joy_data()
                for i in lst:
                    print(i)
                print("===========")
        elif status == RECEIVER_ERROR:
            print("Receiver disconnected. Trying to connect...")
            joy.connect()
            time.sleep(2)
        elif status == JOYSTICK_ERROR:
            print("Joystick disconnected. Please turn it on or connect the receiver...")
            joy.connect()
            joy.read_data()
            time.sleep(2)
        elif status == ACCESS_ERROR:
            print("ERROR: You don't have permission to open this USB-device!")
            break

        time.sleep(0.01)


    




