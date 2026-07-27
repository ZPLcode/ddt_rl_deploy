#!/usr/bin/env python3
"""Keyboard teleop for ddt_rl_deploy (sim2sim & sim2real).

Publishes geometry_msgs/Twist on command/cmd_twist — the topic rl_inference
subscribes for velocity commands.  Key presses step the command; the latest
command is republished at ~20 Hz; exiting always sends a zero command.

    w / s   vx  +/- 0.1 m/s   (forward / back)
    a / d   wz  +/- 0.1 rad/s (turn left / right)
    q / e   vy  +/- 0.1 m/s   (strafe left / right)
    r / f   vz  +/- 0.1       (base-height rate; ignored by policies without it)
    space   stop (all zero)
    x       quit (sends zero)
"""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

STEP = 0.1
LIMITS = {'vx': 1.0, 'vy': 0.5, 'wz': 1.0, 'vz': 0.3}


def clamp(v, lim):
    return max(-lim, min(lim, v))


def main():
    if not sys.stdin.isatty():
        print('teleop needs an interactive terminal (tty)', file=sys.stderr)
        sys.exit(1)
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = Node('teleop_keyboard')
    ns = os.environ.get('ROBOT_NS', '').strip('/')
    topic = (f'{ns}/' if ns else '') + 'command/cmd_twist'
    # Default pub QoS (RELIABLE) is compatible with rl_inference's BEST_EFFORT sub.
    pub = node.create_publisher(Twist, topic, 10)

    vx = vy = wz = vz = 0.0
    print(__doc__)
    print(f'publishing -> {topic}\n')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:   # ~20 Hz loop
                k = sys.stdin.read(1)
                if k == 'w':
                    vx = clamp(vx + STEP, LIMITS['vx'])
                elif k == 's':
                    vx = clamp(vx - STEP, LIMITS['vx'])
                elif k == 'a':
                    wz = clamp(wz + STEP, LIMITS['wz'])
                elif k == 'd':
                    wz = clamp(wz - STEP, LIMITS['wz'])
                elif k == 'q':
                    vy = clamp(vy + STEP, LIMITS['vy'])
                elif k == 'e':
                    vy = clamp(vy - STEP, LIMITS['vy'])
                elif k == 'r':
                    vz = clamp(vz + STEP, LIMITS['vz'])
                elif k == 'f':
                    vz = clamp(vz - STEP, LIMITS['vz'])
                elif k == ' ':
                    vx = vy = wz = vz = 0.0
                elif k in ('x', '\x03'):     # x or Ctrl-C
                    break
                print(f'\r  vx={vx:+.1f}  vy={vy:+.1f}  wz={wz:+.1f}'
                      f'  vz={vz:+.1f}   ', end='', flush=True)
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = vx, vy, vz
            msg.angular.z = wz
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())                 # never leave a velocity latched
        print()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
