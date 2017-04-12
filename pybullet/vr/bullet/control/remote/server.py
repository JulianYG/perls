# Tcp Chat server
# borrowed from http://www.binarytides.com/code-chat-application-server-client-sockets-python/

import socket, select

class Server(object):

    def __init__(self, buffer_size, port_num):

        self.CONNECTION_LIST = []
        self.addr = None

        self._RECV_BUFFER = buffer_size # Advisable to keep it as an exponent of 2
        self._PORT = port_num

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # this has no effect, why ?
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(("0.0.0.0", self._PORT))
        self.server_socket.listen(10)
     
        # Add server socket to the list of readable connections
        self.CONNECTION_LIST.append(self.server_socket)
 
        print "Keyboard server started on port " + str(self._PORT)

    #Function to broadcast chat messages to all connected clients
    def broadcast_data(self, sock, message):
        #Do not send the message to master socket and the client who has send us the message
        for socket in self.CONNECTION_LIST:
            if socket != self.server_socket:
            #if socket != self.server_socket and socket != sock :
                try :
                    socket.send(message)
                except :
                    # broken socket connection may be, chat client pressed ctrl+c for example
                    socket.close()
                    self.CONNECTION_LIST.remove(socket)
    
    def connect(self):
        read_sockets, _, _ = select.select(self.CONNECTION_LIST, [], [])
        for sock in read_sockets:
            #New connection
            if sock == self.server_socket:
                # Handle the case in which there is a new connection recieved through self.server_socket
                sockfd, self.addr = self.server_socket.accept()
                self.CONNECTION_LIST.append(sockfd)
                print "Client (%s, %s) connected" % self.addr
                self.broadcast_data(sockfd, "[%s:%s] Begins simulation \n" % self.addr)
                return 0
        return -1
        
    def poll_event(self):
        
        events = []
        read_sockets, _, _ = select.select(self.CONNECTION_LIST, [], [], 0)
        # Data recieved from client, process it
        # for sock in self.read_sockets:

        for sock in read_sockets:
            try:
            #In Windows, sometimes when a TCP program closes abruptly,
            # a "Connection reset by peer" exception will be thrown
                data = sock.recv(self._RECV_BUFFER)
                print(data)
                if data:
                    self.broadcast_data(sock, 
                        "\r" + '<' + str(sock.getpeername()) + '> ' + data) 
                    key, status = data.split()

                    if status == 'pressed':
                        #TODO: check this part
                        print(key, status)

                        e = {ord(key[-2]): 1}  # 1 KEY_IS_DOWN
                        events.append(e)
                    elif status == 'released':
                        e = {ord(key[-2]): 3}  # 3 KEY_IS_DOWN ??
                        events.append(e)
            except:
                self.broadcast_data(sock, "Client (%s, %s) is offline" % self.addr)
                print "Client (%s, %s) is offline" % self.addr
                sock.close()
                self.CONNECTION_LIST.remove(sock)
                continue

        return events

    def close(self):
        self.server_socket.close()

 
    


