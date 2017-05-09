#!/usr/bin/env python
### A simple script to generate cube objects in simulation

__version__ = "0.1"

from PIL import (Image,
				 ImageDraw,
				 ImageFont
				 )
import StringIO
import sys, os, getopt
from os.path import join as pjoin
from shutil import copyfile
from xml.etree import ElementTree as et

def generate_cube(*args):
	"""
	Takes input: alphabet, red, green, blue channels
	"""
	cube_name = 'cube_{}_{}'.format(args[0],
		'-'.join([str(args[1]), str(args[2]), str(args[3])]))

	font = ImageFont.truetype('/Library/Fonts/Arial.ttf', 200, encoding="unic")
	# text_width, text_height = verdana_font.getsize(alphabet)
	# create a blank canvas with extra space between lines
	canvas = Image.new('RGB', (256, 256), (args[1], args[2], args[3]))

	# draw the text onto the text canvas, and use black as the text color
	draw = ImageDraw.Draw(canvas)
	draw.text((70, 20), unicode(args[0]), font=font, fill="#000000")

	# save the blank canvas to a file
	canvas.save('{}.png'.format(cube_name), 'PNG')

	urdf_name = '{}.urdf'.format(cube_name)
	copyfile('cube_small.urdf', urdf_name)

	new_mtl('cube.mtl', cube_name)
	new_obj('cube.obj', cube_name)

	tree = et.parse(urdf_name)
	element = tree.find('.//mesh')
	element.set('filename', '{}.obj'.format(cube_name))
	tree.write(urdf_name)

def new_mtl(origin, new):
	f1 = open(origin, 'r')
	f2 = open('{}.mtl'.format(new), 'w')
	for line in f1:
		f2.write(line.replace('cube.png', '{}.png'.format(new)))
	f1.close()
	f2.close()

def new_obj(origin, new):

	f1 = open(origin, 'r')
	f2 = open('{}.obj'.format(new), 'w')
	for line in f1:
		f2.write(line.replace('mtllib cube.mtl', 
			'mtllib {}.mtl'.format(new)))
	f1.close()
	f2.close()

def run(argv):

	# Assume 200 users
	r, g, b = 255, 255, 255
	# One minute of data
	a = 'A'	

	try:
		opts, args = getopt.getopt(argv, 
			'a:r:g:b:', ['alphabet=', 'red=', 'green=', 'blue='])
	except getopt.GetoptError:
		sys.exit(2)
	for opt, arg in opts:
		if opt in ('-r', '--red'):
			r = int(arg)
		elif opt in ('-a', '--alphabet'):
			a = arg
		elif opt in ('-g', '--green'):
			g = int(arg)
		elif opt in ('-b', '--blue'):
			b = int(arg)
	generate_cube(a, r, g, b)

if __name__ == '__main__':
	run(sys.argv[1:])

