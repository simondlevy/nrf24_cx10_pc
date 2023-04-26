#!/usr/bin/python3

'''
Serial_test.py

Sends commands to Arduino Uno via serial port to control a drone
using the nRF24L01 wireless boards.

The arrow keys control pitch and roll (forward/reverse and left/right)
and the w,s keys control throttle, and the a,d, keys control the yaw (yaw)

Copyright (c) 2023 perrytsao, Simon D. Levy

'''

import serial
import time
import kbhit


class SerialTester:

    def __init__(self):

        self.throttle = 1000
        self.roll = 1500
        self.pitch = 1500
        self.yaw = 1500

        self.tg = 10
        self.ag = 50
        self.eg = 50
        self.rg = 50

        self.kb = kbhit.KBHit()

        self.arduino = None

    def begin(self):

        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.01)

        time.sleep(1)  # give the connection a second to settle

    def step(self):

        data = self.arduino.readline()

        if data:

            # String responses from Arduino Uno are prefaced with [AU]
            print('[AU]: '+ data.decode())

        if self.kb.kbhit():

            key = ord(self.kb.getch())

            if key == 27:  # ESC
                print('[PC]: ESC exiting')
                return

            elif key == 13:  # Enter
                print('[PC]: Enter')

            elif key == 119:  # w
                self.throttle += self.tg

            elif key == 97:  # a
                self.yaw -= self.rg

            elif key == 115:  # s
                self.throttle -= self.tg

            elif key == 100:  # d
                self.yaw += self.rg

            elif key == 224:  # Special keys (arrows, f keys, ins, del, etc.)
                key = ord(self.kb.getch())
                if key == 80:  # down arrow
                    self.pitch -= self.eg
                elif key == 72:  # up arrow
                    self.pitch += self.eg
                elif key == 77:  # right arroww
                    self.roll += self.ag
                elif key == 75:  # left arrow
                    self.roll -= self.ag

            command = ('%i,%i,%i,%i' %
                       (self.throttle, self.roll, self.pitch, self.yaw))
            # string commands to the Arduino are prefaced with  [PC]
            print('[PC]: '+command)
            self.arduino.write((command + '\n').encode())

    def close(self):

        # close the connection
        self.arduino.close()

        # re-open the serial port which will also reset the Arduino Uno and
        # this forces the quadcopter to power off when the radio loses
        # conection.
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.01)
        self.arduino.close()
        # close it again so it can be reopened the next time it is run.

        self.kb.set_normal_term()


def main():

    tester = SerialTester()

    try:

        tester.begin()

        while True:

            try:
                tester.step()

            except KeyboardInterrupt:

                break

    finally:

        tester.close()


main()
