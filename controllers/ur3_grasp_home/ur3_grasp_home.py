"""Force UR3e to classic HOME and close Robotiq 2F-85 (COFFEE-style API)."""
from controller import Supervisor

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Classic UR fold home — keep this pose
# [pan, lift, elbow, wrist1, wrist2, wrist3]
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
FINGER_CLOSED = 0.75
FINGER_OPEN = 0.0


class UR3GraspHome:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        print("[ur3_grasp_home] started, timestep=", self.timestep)
        print("[ur3_grasp_home] HOME=", HOME)

        self.arm_motors = []
        for name in ARM_JOINTS:
            m = self.robot.getDevice(name)
            if m is None:
                print("[ur3_grasp_home] ERROR missing motor:", name)
                continue
            self.arm_motors.append(m)
            print("[ur3_grasp_home] motor OK:", name)

        # Prefer COFFEE-style explicit names; fall back to device scan
        self.left_finger = self.robot.getDevice(
            "ROBOTIQ 2F-85 Gripper::left finger joint"
        )
        self.right_finger = self.robot.getDevice(
            "ROBOTIQ 2F-85 Gripper::right finger joint"
        )
        self.finger_motors = []
        if self.left_finger and self.right_finger:
            self.finger_motors = [self.left_finger, self.right_finger]
            print("[ur3_grasp_home] finger OK: left/right (explicit names)")
        else:
            for i in range(self.robot.getNumberOfDevices()):
                dev = self.robot.getDeviceByIndex(i)
                n = dev.getName()
                if "finger joint" in n and "sensor" not in n:
                    try:
                        dev.setVelocity(0.8)
                        self.finger_motors.append(dev)
                        print("[ur3_grasp_home] finger OK:", n)
                    except Exception as e:
                        print("[ur3_grasp_home] skip", n, e)
            if len(self.finger_motors) >= 2:
                self.left_finger = self.finger_motors[0]
                self.right_finger = self.finger_motors[1]
            if not self.finger_motors:
                print("[ur3_grasp_home] WARNING: no Robotiq finger joints found")

        for m in self.finger_motors:
            try:
                m.setAvailableTorque(230.0)
            except Exception:
                pass

    def wait(self, seconds):
        steps = int(seconds * 1000 / self.timestep)
        for _ in range(steps):
            if self.robot.step(self.timestep) == -1:
                return

    def move_arm(self, angles, speed_ratio=1.0):
        """Set arm joint targets. speed_ratio in (0.0, 1.0], default full speed."""
        for i, motor in enumerate(self.arm_motors):
            if i >= len(angles):
                break
            try:
                max_speed = motor.getMaxVelocity()
            except Exception:
                max_speed = 2.0
            motor.setVelocity(max_speed * speed_ratio)
            motor.setPosition(angles[i])

    def control_gripper(self, open_gripper=True, quiet=False):
        """Open/close Robotiq 2F-85. Closed angle = FINGER_CLOSED (0.75)."""
        if not self.finger_motors:
            return
        target = FINGER_OPEN if open_gripper else FINGER_CLOSED
        if not quiet:
            if open_gripper:
                print("   (open gripper...)")
            else:
                print("   (close gripper...)")
        for m in self.finger_motors:
            try:
                m.setVelocity(0.8)
            except Exception:
                pass
            m.setPosition(target)

    def run(self):
        print("=== ur3_grasp_home: move to HOME, close gripper ===")
        self.move_arm(HOME, speed_ratio=0.5)
        self.control_gripper(open_gripper=False)
        self.wait(3.0)
        print("[ur3_grasp_home] at HOME, gripper closed — holding")

        # Keep simulation alive and hold targets (quiet after first close)
        while self.robot.step(self.timestep) != -1:
            self.move_arm(HOME, speed_ratio=0.35)
            self.control_gripper(open_gripper=False, quiet=True)


if __name__ == "__main__":
    UR3GraspHome().run()
