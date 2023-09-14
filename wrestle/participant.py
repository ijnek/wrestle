import os
import socket

from . import umsgpack

# Enable Msgpack Old Specification Compatibility Mode, which is used in LoLA
umsgpack.compatibility = True

class Participant:
    SOCK_PATH = "/tmp/robocup"
    TCP_BASE_PORT = 10000
    DOF = 25
    PHALANX_MAX = 8
    ACTUATOR_PKT_SIZE = 786
    MSGPACK_READ_LENGTH = 896
    conn = None

    sensors = {
        "Stiffness": [ 1.0 ] * DOF,
        "Position": [ 0.0 ] * DOF,
        "Temperature": [  0.0 ] * DOF,
        "Current": [  0.0 ] * DOF,
        "Battery": [ 1.0, -32708.0, 0.0, 0.0 ],
        "Accelerometer": [ 0.0, 0.0, 0.0 ],
        "Gyroscope": [ 0.0, 0.0, 0.0 ],
        "Angles": [ 0.0, 0.0 ],
        "Sonar": [ 0.0, 0.0 ],
        "FSR": [  0.0 ] * 8,
        "Status": [ 0 ] * DOF,
        "Touch": [ 0.0 ] * 14,
        "RobotConfig": [ "P0000000000000000000", "6.0.0", "P0000000000000000000", "6.0.0" ],
    }

    def init(self, webots_node, properties):
        # we get the robot instance from the webots_node
        self.__robot = webots_node.robot

        # initialize stuff
        self.findAndEnableDevices()

        self.setupSocket()

    def setupSocket(self):
        try:
            os.unlink(self.SOCK_PATH)
        except OSError:
            if os.path.exists(self.SOCK_PATH):
                raise

        sock_type = socket.AF_UNIX
        self.sock = socket.socket(sock_type, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.SOCK_PATH)
        self.sock.listen()
        self.sock.settimeout((self.timeStep-1)/1000.0)

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.__robot.getBasicTimeStep())

        self.us = []
        self.fsr = []
        self.leds = {}
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        self.pos = []
        self.motors = []

        self.accelerometer = self.__robot.getDevice('accelerometer')
        self.gyro = self.__robot.getDevice('gyro')
        # self.inertialUnit = self.__robot.getDevice('inertial unit')

        for i in ['Sonar/Left', 'Sonar/Right']:
            self.us.append(self.__robot.getDevice(i))

        for i in ['LFsr', 'RFsr']:
            self.fsr.append(self.__robot.getDevice(i))

        self.lfootlbumper = self.__robot.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.__robot.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.__robot.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.__robot.getDevice('RFoot/Bumper/Right')

        self.leds["Chest"] = self.__robot.getDevice('ChestBoard/Led')
        self.leds["REar"] = self.__robot.getDevice('Ears/Led/Right')
        self.leds["LEar"] = self.__robot.getDevice('Ears/Led/Left')
        self.leds["LEye"] = self.__robot.getDevice('Face/Led/Left')
        self.leds["REye"] = self.__robot.getDevice('Face/Led/Right')
        self.leds["LFoot"] = self.__robot.getDevice('LFoot/Led')
        self.leds["RFoot"] = self.__robot.getDevice('RFoot/Led')

        # finger motors
        for i in range(self.PHALANX_MAX):
            self.lphalanx.append(self.__robot.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.__robot.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # get motors
        for j in [ "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll",
                    "LElbowYaw", "LElbowRoll", "LWristYaw", "LHipYawPitch", "LHipRoll",
                    "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipRoll",
                    "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "RShoulderPitch",
                    "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LPhalanx1", "RPhalanx1", "RHipYawPitch" ]:
            self.pos.append(self.__robot.getDevice(j+"S"))
            self.motors.append(self.__robot.getDevice(j))

        # enable devices
        self.accelerometer.enable(self.timeStep)
        self.gyro.enable(self.timeStep)
        # self.inertialUnit.enable(self.timeStep)

        for s in self.us:
            s.enable(self.timeStep)

        for f in self.fsr:
            f.enable(self.timeStep)

        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        for s in self.pos:
            s.enable(self.timeStep)

        self.key = -1
        self.keyboard = self.__robot.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def step(self):
        if self.conn:
            self.updateSensors()
            try:
                # send sensor data to LoLa client
                packed = umsgpack.packb(self.sensors, force_float_precision="single")
                if len(packed) != self.MSGPACK_READ_LENGTH:
                    print("Msgpack packet size doesn't match LoLA specifications." )
                self.conn.send(packed)
                data = self.conn.recv(self.ACTUATOR_PKT_SIZE*3)
                if data:
                    self.updateActuators(umsgpack.unpackb(data))
            except TimeoutError:
                print("Timeout while waiting for LoLa actuators.")
            except ConnectionError:
                self.conn.close()
                self.conn = None
                print("LoLa client disconnected.")
        else:
            try:
                (self.conn, addr) = self.sock.accept()
                packed = umsgpack.packb(self.sensors, force_float_precision="single")
                if len(packed) != self.MSGPACK_READ_LENGTH:
                    print("Msgpack packet size doesn't match LoLA specifications." )
                self.conn.send(packed)
                print("LoLa client connected.")
            except:
                self.conn = None


    def updateSensors(self):
        # IMU
        a = self.accelerometer.getValues()
        self.sensors["Accelerometer"] = [ a[0], a[1], -a[2] ]
        g = self.gyro.getValues()
        self.sensors["Gyroscope"] = [ g[0], g[1], g[2] ]
        # imu = self.inertialUnit.getRollPitchYaw()
        # self.sensors["Angles"] = [ imu[0], imu[1] ]

        # motors
        for i in range(self.DOF):
            self.sensors["Position"][i] = self.pos[i].getValue()

        # sonar
        self.sensors["Sonar"] = [self.us[0].getValue(), self.us[1].getValue()]

        # foot bumpers
        self.sensors["Touch"][4] = self.lfootlbumper.getValue()
        self.sensors["Touch"][5] = self.lfootrbumper.getValue()
        self.sensors["Touch"][9] = self.rfootlbumper.getValue()
        self.sensors["Touch"][10] = self.rfootrbumper.getValue()

        # FSR
        # conversion is stolen from webots nao_demo_python controller
        fsv = [self.fsr[0].getValues(), self.fsr[1].getValues()]

        a = []
        a.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        a.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        a.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        a.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        a.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        a.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        a.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        a.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left
        for i in range(len(a)):
            self.sensors["FSR"][i] = max(0.0, a[i]/25.0) # fix scaling of values

    def setHands(self, langle, rangle):
        self.setHand(langle, False)
        self.setHand(rangle, True)

    def setHand(self, angle, right=True):
        array = self.rphalanx
        if not right:
            array = self.lphalanx

        for i in range(self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(array) > i and array[i] is not None:
                array[i].setPosition(clampedAngle)


    def led_array2RGB(self, a):
        for i in range(len(a)):
            a[i] = max(0.0, min(a[i], 1.0))
        return int(a[0]*255)<<16 | int(a[1]*255)<<8 | int(a[2]*255)

    def updateActuators(self, actuators):
        # motors
        if b"Position" in actuators:
            for i in range(self.DOF-2):
                self.motors[i].setPosition(actuators[b"Position"][i])
            # set hands, since they have more than 1 actuator each in webots
            self.setHands(actuators[b"Position"][23], actuators[b"Position"][24])
            # set RHipYawPitch which does not exists in LoLa packet
            self.motors[25].setPosition(actuators[b"Position"][7])

        # LEDs
        if b"Chest" in actuators:
            self.leds["Chest"].set(self.led_array2RGB(actuators[b"Chest"]))
        if b"LFoot" in actuators:
            self.leds["LFoot"].set(self.led_array2RGB(actuators[b"LFoot"]))
        if b"RFoot" in actuators:
            self.leds["RFoot"].set(self.led_array2RGB(actuators[b"RFoot"]))

        # webots model has only one LED per eye, so just use the first one
        if b"LEye" in actuators:
            v = actuators[b"LEye"]
            self.leds["LEye"].set(self.led_array2RGB([v[0], v[8], v[16]]))
        if b"REye" in actuators:
            v = actuators[b"REye"]
            self.leds["REye"].set(self.led_array2RGB([v[0], v[8], v[16]]))

        # webots model has only one LED per ear, so just use the first one
        if b"LEar" in actuators:
            self.leds["LEar"].set(self.led_array2RGB([0, 0, actuators[b"LEar"][0]]))
        if b"REar" in actuators:
            self.leds["REar"].set(self.led_array2RGB([0, 0, actuators[b"REar"][0]]))


    # def init(self, webots_node, properties):
    #     # we get the robot instance from the webots_node
    #     self.__robot = webots_node.robot

    #     # we initialize the shoulder pitch motors using the Robot.getDevice() function:
    #     self.__RShoulderPitch = self.__robot.getDevice("RShoulderPitch")
    #     self.__LShoulderPitch = self.__robot.getDevice("LShoulderPitch")

    #     # to control a motor, we use the setPosition() function:
    #     self.__RShoulderPitch.setPosition(1.3)
    #     self.__LShoulderPitch.setPosition(1.3)
    #     # for more motor control functions, see the documentation: https://cyberbotics.com/doc/reference/motor
    #     # to see the list of available devices, see the NAO documentation: https://cyberbotics.com/doc/guide/nao

    #     rclpy.init(args=None)
    #     self.__node = rclpy.create_node('nao_driver')

    # def step(self):
    #     # Mandatory function to go to the next simulation step
    #     rclpy.spin_once(self.__node, timeout_sec=0)

    #     if self.__robot.getTime() == 1: # We wait a bit for the robot to stabilise
    #         # to play a motion from the library, we use the play() function as follows:
    #         # self.__library.play('Forwards50')
    #         pass
