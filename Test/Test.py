import time

from Multiwii.Multiwii import MultiWii


class Test:

    def __init__(self, serial_port, udp_communication, ip_address, port):
        self.mw = MultiWii()
        self.mw.settings.serial_port = serial_port
        self.mw.settings.udp_communication = udp_communication
        self.mw.settings.ip_address = ip_address
        self.mw.settings.port = port

    # self.altitude = {'estalt': 0, 'vario': 0, 'elapsed': 0, 'timestamp': 0}
    def test_altitude(self):

        altitude = self.mw.get_altitude()
        print("-----ALTITUDE-----\n")
        print("EstAlt: {} cm".format(altitude["estalt"]))
        print("Vario: {} cm/s".format(altitude["vario"]))
        print("elapsed: {}".format(altitude["elapsed"]))
        print("timestamp: {}".format(altitude["timestamp"]))
        print("-------------------\n")

    # self.raw_imu = {'accx': 0, 'accy': 0, 'accz': 0, 'gyrx': 0, 'gyry': 0, 'gyrz': 0, 'magx': 0, 'magy': 0,
    #                   'magz': 0, 'elapsed': 0, 'timestamp': 0}

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

def main():
    tests = Test("COM5", False, "", "")

    while 1:
        #tests.test_altitude()
        #time.sleep(1)
        tests.test_raw_imu()
        tests.test_altitude()
        tests.test_set_rc()
        time.sleep(0.04)
        #tests.test_telemetry()



if __name__ == '__main__':
    main()
