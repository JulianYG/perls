import redis

def my_handler(msg):
	print('my handler: ', msg['data'])

r = redis.StrictRedis(host='localhost', port=6379, db=0)

p = r.pubsub()

p.subscribe(**{'channel1': my_handler})

for _ in p.listen():
	pass
