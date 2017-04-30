
from bullet.control.sockets import *

ip = '172.24.68.111'

host = redis_socket.RedisSocket(ip)

host.connect_with_client()

while 1: 

	events = 
