Import("env")

from os.path import join


framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
camera_sdk_dir = join(framework_dir, "tools", "sdk", "esp32s3")

env.AppendUnique(
    CPPPATH=[
        join(camera_sdk_dir, "include", "esp32-camera", "driver", "include"),
        join(camera_sdk_dir, "include", "esp32-camera", "conversions", "include"),
    ],
    LIBPATH=[join(camera_sdk_dir, "lib")],
    LIBS=["esp32-camera"],
)
