
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../src'))

from lib.control import Controller as sc

# Prevent recursively spawning subprocesses
if __name__ == '__main__':

	s = sc('config.xml')
# s.start_all()

	s.start()
