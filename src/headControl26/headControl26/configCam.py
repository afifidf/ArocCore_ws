import subprocess

class CameraConfig:
    def __init__(self, device="/dev/video0"):
        self.device = device

    def set_param(self, name, value):
        try:
            subprocess.run([
                "v4l2-ctl",
                "-d", self.device,
                "-c", f"{name}={value}"
            ], check=True)
            print(f"[CameraConfig] Set {name} = {value}")
        except subprocess.CalledProcessError:
            print(f"[CameraConfig] Failed to set {name}")

    def apply_default_settings(self):
        # Matikan auto exposure dulu
        self.set_param("exposure_auto", 1)

        # Setting manual
        self.set_param("brightness", 128)
        self.set_param("contrast", 32)
        self.set_param("saturation", 64)
        self.set_param("sharpness", 3)
        self.set_param("gain", 50)
        self.set_param("exposure_absolute", 200)
