#!/usr/bin/env python
### A simple script to generate tray box objects in simulation

__version__ = "0.1"

import sys, os, getopt
from shutil import copyfile
from xml.etree import ElementTree as et

def generate_cube(*args):
	"""
	Takes input: alphabet, red, green, blue channels
	"""
	tray_name = 'tray_{}'.format('-'.join([args[0], 
		args[1], args[2]]))

	urdf_name = '{}.urdf'.format(tray_name)
	copyfile('traybox.urdf', urdf_name)

	tree = et.parse(urdf_name)
	element = tree.find('.//color')
	element.set('rgba', '{} {} {} 1'.format(*args))
	tree.write(urdf_name)

def run(argv):

	# Assume 200 users
	r, g, b = 255, 255, 255
	# One minute of data
	try:
		opts, args = getopt.getopt(argv, 
			'r:g:b:', ['red=', 'green=', 'blue='])
	except getopt.GetoptError:
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-r', '--red'):
			r = arg
		elif opt in ('-g', '--green'):
			g = arg
		elif opt in ('-b', '--blue'):
			b = arg
	generate_cube(r, g, b)

if __name__ == '__main__':
	run(sys.argv[1:])

