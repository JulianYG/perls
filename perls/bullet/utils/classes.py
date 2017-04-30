

class IllegalOperation(Exception):
	def __init__(self, link):
		self.link = link

class WARNING_HOOK(object):
	def __init__(self, c):
		self.content = c



		