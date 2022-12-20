import curses
import time
import sys
import erpc
from bbcar_control import *

def main():

    """
    The curses.wrapper function is an optional function that
    encapsulates a number of lower-level setup and teardown
    functions, and takes a single function to run when
    the initializations have taken place.
    """

    if len(sys.argv) != 2:
        print("Usage: python bbcar_control.py <serial port to use>")
        exit()

    # Initialize all erpc infrastructure
    global client
    xport = erpc.transport.SerialTransport(sys.argv[1], 9600)
    client_mgr = erpc.client.ClientManager(xport, erpc.basic_codec.BasicCodec)
    client = client.BBCarServiceClient(client_mgr)

    curses.wrapper(curses_main)


def curses_main(w):

    """
    This function is called curses_main to emphasise that it is
    the logical if not actual main function, called by curses.wrapper.
    """

    w.addstr("----------------------------------\n")
    w.addstr("| Use arrow keys to control car. |\n")
    w.addstr("| s to stop car and q to exit.   |\n")
    w.addstr("---------------------------------\n")
    w.refresh()

    bbcar_control(w)


def bbcar_control(w):
    w.nodelay(True)
    while True:
        char = w.getch()
        w.move(5, 0)
        w.clrtobot()
        if char == ord('q'): break  # q
        elif char == curses.KEY_RIGHT: 
           w.addstr("Turn right.")
           w.refresh()
           client.turn(1, 40, -0.3)
        elif char == curses.KEY_LEFT: 
           w.addstr("Turn left.")
           w.refresh()
           client.turn(1, 40, 0.3)
        elif char == curses.KEY_UP: 
           w.addstr("Go straight.")
           w.refresh()
           client.goStraight(1, 100)
        elif char == curses.KEY_DOWN: 
           w.addstr("Go backward.")
           w.refresh()
           client.goStraight(1, -100)
        elif char == ord('d'): 
           w.addstr("Detect.")
           w.refresh()
           client.detect(1, 100, 0.3)
        elif char == ord('s'): 
           w.addstr("Stop.")
           w.refresh()
           client.stop(1)
        else: pass
        time.sleep(0.1)

main()