import json

def get_distance(posA, posB):
	dist = 0.
	for i in range(len(posA)):
		dist += (posA[i] - posB[i]) ** 2
	return dist

def read_config(config):
	dic = {}
	with open(config, 'r') as f:
		config = json.load(f)
		for k, v in config.items():
			dic[str(k)] = v
	return dic

def parse_log(filename, verbose=True):

	  	f = open(filename, 'rb')
	  	print('Opened'),
	  	print(filename)

	  	keys = f.readline().decode('utf8').rstrip('\n').split(',')
	  	fmt = f.readline().decode('utf8').rstrip('\n')

	  	# The byte number of one record
	  	sz = struct.calcsize(fmt)
	  	# The type number of one record
	  	ncols = len(fmt)

	  	if verbose:
	  		print('Keys:'), 
	  		print(keys)
	  		print('Format:'),
	  		print(fmt)
	  		print('Size:'),
	  		print(sz)
	  		print('Columns:'),
	  		print(ncols)

	  	# Read data
	  	wholeFile = f.read()
	  	# split by alignment word
	  	chunks = wholeFile.split(b'\xaa\xbb')
	  	log = list()
	  	for chunk in chunks:
		    if len(chunk) == sz:
		      	values = struct.unpack(fmt, chunk)
		      	record = list()
		      	for i in range(ncols):
		        	record.append(values[i])
		      	log.append(record)

	  	return log

