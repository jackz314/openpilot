import mmap
import struct

def create_files():
    import os
    os.umask(0)
    try:
        os.mkdir("/dev/shm/SCS", mode=0o777)
    except: # already exists, change permission
        os.chmod("/dev/shm/SCS", 0o777)
    os.close(os.open("/dev/shm/SCS/SCSTelemetry",os.O_CREAT | os.O_RDWR, 0o777))
    os.chmod("/dev/shm/SCS/SCSTelemetry", 0o777)
    with open("/dev/shm/SCS/SCSTelemetry", "wb+") as fd:
        fd.write(b"\0"*22420) # size of telemetry struct
        fd.flush()

class scssdkclient:
    def __init__(self):
        try:
            self.fd = open("/dev/shm/SCS/SCSTelemetry")
        except: # file probably doesn't exist
            create_files()
            self.fd = open("/dev/shm/SCS/SCSTelemetry")

    def __del__(self):
        self.fd.close()

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
