# telnet program example

# modified from http://www.binarytides.com/code-chat-application-server-client-sockets-python/

import socket, select, string, sys
from pynput.keyboard import Key, Listener, Controller
import Queue 

def prompt() :
    sys.stdout.write('<You> ')
    sys.stdout.flush()
 
#main function
if __name__ == "__main__":

    if(len(sys.argv) < 3) :
        print 'Usage : python telnet.py hostname port'
        sys.exit()
     
    host = sys.argv[1]
    port = int(sys.argv[2])
     
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
     
    # connect to remote host
    try :
        s.connect((host, port))
    except :
        print 'Unable to connect'
        sys.exit()
     
    print 'Connected to remote host. Start sending messages'
    #prompt()

    def on_press(key):
        key_str = '{0} pressed'.format(key)
        print(key_str)
        s.send(key_str)

    def on_release(key):
        key_str = '{0} released'.format(key)
        print(key_str)
        s.send(key_str)
        if key == Key.esc:
            # Stop listener
            return False

    with Listener(on_press=on_press, on_release=on_release) as listener:
        while 1:
            #socket_list = [sys.stdin, s]
            socket_list = [s]
             
            # Get the list sockets which are readable
            read_sockets, write_sockets, error_sockets = select.select(socket_list , [], [])

            for sock in read_sockets:
                #incoming message from remote server
                if sock == s:
                    data = sock.recv(4096)
                    if not data :
                        print '\nDisconnected from chat server'
                        listener.join()
                        sys.exit()
                    else :
                        print data
                        #sys.stdout.write(data)
                        #prompt()
                 
                # #user entered a message
                # else :
                #     msg = sys.stdin.readline()
                #     s.send(msg)
                #     prompt()


