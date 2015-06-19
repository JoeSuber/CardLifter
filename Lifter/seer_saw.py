#!/usr/bin python

import serial
import sys


def hookup(port="/dev/ttyUSB0", rate=115200, tout=0):
    return serial.Serial(port=port, baudrate=rate, timeout=tout)


def getkeys():
    print("type a letter: ")
    return raw_input()


def flushread(con=None):
    if not con:
        return None
    inflo = {}
    if con.isOpen():
        chatter = con.readline().strip('\x00').strip()
        chatline = 0
        while chatter:
            inflo[chatline] = chatter
            print("{:4} : {}".format(chatline, chatter))
            chatline += 1
            chatter = con.readline().strip('\x00').strip()
        return inflo
    return None


def main():
    print("enter 'q' to quit this madness")
    print("platform is: {}".format(sys.platform))
    con = hookup()
    if flushread(con=con):
        print("(opened) {}".format(con.getPort()))
    else:
        print("failed to open the port. exiting main()")
        return 0
    while True:
        keypress = getkeys().strip()
        if keypress > "":
            con.write(keypress)
            print("sent: {}".format(keypress))
        if keypress == "q":
            return 0
        latest_news = flushread(con=con)

if __name__ == "__main__":
    exit(main())
