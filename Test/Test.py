import socket
import time
import _thread
import struct


from Multiwii.Multiwii import MultiWii



class Test:

    def __init__(self):
        self.mw = MultiWii()

    def test_altitude(self):

        altitude = self.mw.get_altitude()
        print("-----ALTITUDE-----\n")
        print("EstAlt: {} cm".format(altitude["estalt"]))
        print("Vario: {} cm/s".format(altitude["vario"]))
        print("elapsed: {}".format(altitude["elapsed"]))
        print("timestamp: {}".format(altitude["timestamp"]))
        print("-------------------\n")

    def test_raw_imu(self):

        raw_imu = self.mw.get_raw_imu()

        print("-----RAW_IMU-----")
        print("accx: {} ,".format(raw_imu["accx"]))
        print("accy: {} ,".format(raw_imu["accy"]))
        print("accz: {} ,".format(raw_imu["accz"]))
        print("gyrx: {} ,".format(raw_imu["gyrx"]))
        print("gyry: {} ,".format(raw_imu["gyry"]))
        print("gyrz: {} ".format(raw_imu["gyrz"]))
        print("-------------------\n")

    def test_set_rc(self):

        roll = 1560
        pitch = 1560
        yaw = 1700
        throttle = 1800

        self.mw.set_rc([roll, pitch, yaw, throttle])
        rc = self.mw.get_rc()

        print("-----RC-----\n")
        print("Roll: {}".format(rc["roll"]))
        print("Pitch: {}".format(rc["pitch"]))
        print("Yaw: {}".format(rc["yaw"]))
        print("Throttle: {}".format(rc["throttle"]))
        print("-------------------\n")

    def test_telemetry(self):

        self.mw.telemetry_loop()
    '''
        if self.mw.settings.MSP_ALTITUDE:
            alt = self.mw.drone.altitude
            print("-------Altitude--------\n")
            print("EstAlt: %f cm, Vario: %f") % (alt["estalt"], alt["vario"])
            print("-----------------------\n")

        if self.mw.settings.MSP_RAW_IMU:
            raw_imu = self.mw.drone.raw_imu
            print("------Raw_IMU-------")
            print("ACC -> accx: %f, accy: %f, accz %f" % (raw_imu["accx"], raw_imu["accy"], raw_imu["accz"]))
            print("GYRO -> gyrx: %f, gyry: %f, gyrz: %f" % (raw_imu["gyrx"], raw_imu["gyry"], raw_imu["gyrz"]))
            print("---------------------")

        if self.mw.settings.MSP_RC:
            rc = self.mw.drone.rc_channels
            print("--------Rc--------")
            print("Roll: %f, Pitch: %f, Yaw: %f, Throttle: %f" % (rc["roll"], rc["pitch"], rc["yaw"], rc["throttle"]))
            print("------------------")
            
        '''

    def test_udp_telemetry(self):

        server_started = False

        try:
            address = (self.mw.settings.ip_address, 4446)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print("Socket creation: Socket created!")
            sock.bind(address)
            print("Socket binding: Socket bound!")
            server_started = True

        except socket.error as err:
            print("Error starting server: {}".format(err))

        if server_started:

            t = _thread.start_new_thread(self.mw.udp_telemetry_loop, ())

            if not t:
                print("Error: MultiWii server not started")
            else:
                time_start = time.time()
                timer = time.time()

                while time_start - time.time() < 10:

                    if time.time() - timer >= self.mw.settings.TELEMETRY_TIME:

                        data = sock.recv(40)

                        print("Received {} ".format(len(data)))
                        code = struct.unpack('<h', data[:2])[0]
                        size = struct.unpack('<h', data[2:4])[0]
                        data = struct.unpack('<' + 'h' * int(size / 2), data[4:size + 4])

                        print("Code: {0} Size: {1} Data: {2}".format(code, size, data))

def main():
    tests = Test()

    tests.test_udp_telemetry()




if __name__ == '__main__':
    main()
