"""Keyboard jog for UR5e — tune HOME / APPROACH / INSERT for center hole."""
from controller import Robot

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Theo universal_robots/controllers/clone/poses.py (HOME / PICK_ABOVE / PICK_DOWN)
HOME = [0.0, -1.570, 1.570, -1.570, -1.570, 0.0]
APPROACH = [0.0, -1.210, 1.370, -1.770, -1.590, 0.0]
INSERT = [0.0, -1.010, 1.590, -2.170, -1.553, -0.020]


def main():
    robot = Robot()
    dt = int(robot.getBasicTimeStep())

    kb = robot.getKeyboard()
    kb.enable(dt)

    motors = []
    for name in JOINT_NAMES:
        m = robot.getDevice(name)
        m.setVelocity(0.8)
        motors.append(m)

    target = list(HOME)
    selected = 0

    def apply():
        for i, m in enumerate(motors):
            m.setPosition(target[i])

    apply()
    print('Phim: 1-6 chon khop | Z/X giam/tang | H home | A approach | I insert | R retract')
    print('HOME=', HOME)
    print('APPROACH=', APPROACH)
    print('INSERT=', INSERT)

    while robot.step(dt) != -1:
        key = kb.getKey()
        while key != -1:
            if ord('1') <= key <= ord('6'):
                selected = key - ord('1')
                print('Selected', selected + 1, JOINT_NAMES[selected], 'val', round(target[selected], 4))
            elif key in (ord('z'), ord('Z')):
                target[selected] -= 0.02
                apply()
                print('target', [round(v, 4) for v in target])
            elif key in (ord('x'), ord('X')):
                target[selected] += 0.02
                apply()
                print('target', [round(v, 4) for v in target])
            elif key in (ord('h'), ord('H')):
                target = list(HOME)
                apply()
                print('-> HOME')
            elif key in (ord('a'), ord('A')):
                target = list(APPROACH)
                apply()
                print('-> APPROACH')
            elif key in (ord('i'), ord('I')):
                target = list(INSERT)
                apply()
                print('-> INSERT')
            elif key in (ord('r'), ord('R')):
                target = list(APPROACH)
                apply()
                print('-> RETRACT (approach)')
            elif key in (ord('p'), ord('P')):
                print('COPY poses:')
                print('HOME =', [round(v, 4) for v in target])
            key = kb.getKey()


if __name__ == '__main__':
    main()
