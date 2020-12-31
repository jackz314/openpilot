openpilot in Euro Truck Simulator 2 / American Truck Simulator
=====================

## Running OP

Start the bridge and openpilot.
```
./launch_openpilot.sh
./bridge.py
```

## Control Flow/Structure

We capture in-game frames at a high speed via [NvFBC](https://developer.nvidia.com/capture-sdk), this ensures quick capture and low overhead and latency. We then send them over to OP via cereal's messaging, this is similar to the carla version but in C++. All capture stuff is in the [cap](cap) folder and everything is mainly done in [nvfbc_cap.cc](cap/nvfbc_cap.cc), if you'd like to swap out NvFBC for something else (e.g. X11's [xshm](https://linux.die.net/man/3/xshm) or [mss](https://github.com/BoboTiG/python-mss)) but keep the messaging stuff, the messaging part is done in the `send_frame` function.

The [bridge](bridge.py) is based on carla's [bridge](../sim/bridge.py), with some tweaks and emulation to adapt for truck sim. 

I've also included integration with Euro Truck Simulator 2 / American Truck Simulator's SDK, this is done with [scssdk.py](scssdk.py) and the [scs-sdk-plugin](https://github.com/jackz314/scs-sdk-plugin) (based on the official [SCS SDK](https://modding.scssoft.com/wiki/Documentation/Engine/SDK/Telemetry)), which allows telemetry data like speed, acceleration, and game/cruise control status to be fed back to OP and potentially used to control it. For more details on the plugin or the SDK, see the scs-sdk-plugin](https://github.com/jackz314/scs-sdk-plugin) repo.

## Controls

OP sends control signals back to truck sim with a virtual joystick found in [joystick.py](joystick.py), you may need to tweak control settings in truck sim to make it work (should all be inverted). After starting the bridge, you can control OP by pressing "C" to engage and increase cruise speed, press "Z" to decrease cruise speed, and press "V" to cancel OP and disengage.

Stock auto lane change is supported, initiate a lane change like you would on a real car by turning on the corresponding turning indicator lights (usually '[' and ']' in truck sim). The only difference from OP in a real car is that you don't need to additionally steer left or right to start the lane change, it would start automatically shortly after the indicators are on.

## Notes

I have found that in truck sim, using OP with viewing perspective 6 works the best, 1 and 2 also kinda work but they perform worse than 6. 

Setting screen capture framerate to 20 FPS seems to work pretty smoothly, and sometimes performs better than higher framerates, so I'd start there and tweak if needed. 

OP works pretty nicely now in truck sim, but occasionally it still under or over steer when the turn is too tight, to improve this, you can tweak the steering ratio and the max steering angle. Dial them down if OP is under-steering (turns are too wide), and push them up if OP is over-steering (turns are too tight). Currently there is a simple dynamic steer ratio set based on the speed ()

Sometimes, OP would fail to read from the ONNX model for some unknown reason, in this case just restart either bridge.py or OP until it works. When OP fails to read, you'll see the modeld text in the log turn red.

## Future Improvements

Better tweaks for steering ratio and max steering angle.

More control integration with truck sim, things like engaging/disengaging OP based on in-game input and in-game cruise control status. These data fields are all available in [scssdk.py](scssdk.py).

Windows support? This will depend largely on the status of [WSL](https://docs.microsoft.com/en-us/windows/wsl/), particularly [support for OpenGL & OpenCL](https://devblogs.microsoft.com/directx/in-the-works-opencl-and-opengl-mapping-layers-to-directx/) via [DirectX mapping layers](https://devblogs.microsoft.com/directx/in-the-works-opencl-and-opengl-mapping-layers-to-directx/). Or perhaps someone would be able to port openpilot to Windows, that would be awesome but it'd be very difficult.
