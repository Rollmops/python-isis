__author__ = 'tuerke'

from .. import data

def copy_geom(reference_image, images, attributes=None):
	if(attributes == None):
		attributes=['indexOrigin','voxelSize','voxelGap','rowVec','columnVec','sliceVec']
	for image in images:
		for attribute in attributes:
			image.setProperty(attribute,reference_image.getProperty(attribute))