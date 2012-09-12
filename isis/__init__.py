from log import log
import util
import data
import tools

__all__=['util','data','tools', 'log', 'getVersion', 'scientific']

VERSION = '0.15.0'
VERSION_AS_LIST = tuple(map(int, VERSION.split('.')))

def getVersion():
	return VERSION_AS_LIST