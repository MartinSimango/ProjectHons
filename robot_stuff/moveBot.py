import wifibot
import sys

import time
SPEED = int(sys.argv[2])
bot= wifibot.Wifibot(sys.argv[1]) 



#code adapted from https://stackoverflow.com/questions/22397289/finding-the-values-of-the-arrow-keys-in-python-why-are-they-triples
import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno() #get stdin file descripor
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                print(k)
                if k!='':break
        if k=='\x1b[A':
            print("up") 
            bot.move(SPEED,SPEED)
            return True
        elif k=='\x1b[B':
            print("down")
            bot.move(-SPEED,-SPEED)
            return True
        elif k=='\x1b[C':
            print ("right")
            bot.move(SPEED,-SPEED)
            return True
        elif k=='\x1b[D':
            print ("left")
            bot.move(-SPEED,SPEED)
            return True
        else: 
            print("not an arrow key!")
            return False
        return True
print("Start!")
while(True):
    if(not get()):
        break
    #time.sleep(0.05)
