# note: run in python3!
from postprocess_pickle import readFile
from six.moves import cPickle as pickle
from glob import glob

def saveFile(fname, data):
    f = open(fname, 'wb')
    pickle.dump(data, f, protocol=2)
    f.close()


for fname in glob("success/*.bin"):
    data = readFile(fname)
    saveFile("success_new/" + fname.split('/')[-1], data)

