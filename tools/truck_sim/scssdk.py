import mmap
import struct


class scssdkclient:
    def __init__(self):
        self.fd = open("/dev/shm/SCS/SCSTelemetry")

    # noinspection PyAttributeOutsideInit
    def update(self):
        self.mm = mmap.mmap(self.fd.fileno(), length=0, flags=mmap.MAP_SHARED, access=mmap.ACCESS_READ)

        self.sdkActive = struct.unpack_from("?", self.mm, 0)[0]
        self.paused = struct.unpack_from("?", self.mm, 4)[0]
        self.time = struct.unpack_from("Q", self.mm, 8)[0]

        # skip middle fields, +248 to start from speed
        self.speed, self.engineRpm, self.userSteer, self.userThrottle, self.userBrake, self.userClutch, \
        self.gameSteer, self.gameThrottle, self.gameBrake, self.gameClutch, self.cruiseControlSpeed, \
            = struct.unpack_from("11f", self.mm, 700 + 248)

        self.speedLimit = struct.unpack_from("11f", self.mm, 700 + 248 + 120)[0]  # skip middle fields

        self.parkBrake, self.motorBrake, self.airPressureWarning, self.airPressureEmergency, self.fuelWarning, \
        self.adblueWarning, self.oilPressureWarning, self.waterTemperatureWarning, self.batteryVoltageWarning, \
        self.electricEnabled, self.engineEnabled, self.wipers, self.blinkerLeftActive, self.blinkerRightActive, \
        self.blinkerLeftOn, self.blinkerRightOn, self.lightsParking, self.lightsBeamLow, self.lightsBeamHigh, \
        self.lightsBeacon, self.lightsBrake, self.lightsReverse, self.cruiseControl \
            = struct.unpack_from("23?", self.mm, 1500 + 66)

        self.lv_accelerationX, self.lv_accelerationY, self.lv_accelerationZ, self.av_accelerationX, \
        self.av_accelerationY, self.av_accelerationZ, self.accelerationX, self.accelerationY, self.accelerationZ, \
        self.aa_accelerationX, self.aa_accelerationY, self.aa_accelerationZ \
            = struct.unpack_from("12f", self.mm, 1640 + 228)

        self.mm.close()

    def printall(self):
        attrs = vars(self)
        # now dump this in some way or another
        print(', \n'.join("%s: %s" % item for item in attrs.items()))
