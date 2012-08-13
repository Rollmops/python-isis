__author__ = 'tuerke'

from .. import data, util

def swap_axes( image, axis1, axis2, adapt_orientation = True ):
	'''Swaps 'axis1' and 'axis2' of the image 'image' in image space.

	If 'adapt_orientation' is set to 'True' it will also swap:

	indexOrigin[axis1] <--> indexOrigin[axis2]

	and the respective orientation vectors represented by 'axis1' and 'axis2'.
	That means if the orientation was correct before using 'swap_axes'
	it is recommended to set 'adapt_orientation' to 'True'.

	If 'adapt_orientation' is set to 'False' the orientation will
	be preserved.

	Returns the swapped image.
	'''	
	ndarray = image.getArray()
	swapped_ndarray = ndarray.swapaxes( axis1, axis2 )
	swapped_image = data.Image( swapped_ndarray.copy(), image )
	
	#adapt orientation
	if( adapt_orientation == True and axis1 != data.dimensions.TIME_DIM and axis2 != data.dimensions.TIME_DIM  ):
		vectors = [ image.getProperty("rowVec"), image.getProperty("columnVec"), image.getProperty("sliceVec") ]
		index_origin = image.getProperty( "indexOrigin" )
		voxel_size= image.getProperty("voxelSize")

		vectors[axis1], vectors[axis2] = vectors[axis2], vectors[axis1]

		index_origin[axis1], index_origin[axis2] = index_origin[axis2], index_origin[axis1]

		voxel_size[axis1], voxel_size[axis2] = voxel_size[axis2], voxel_size[axis1]

		swapped_image.setProperty( "rowVec", vectors[0]  )
		swapped_image.setProperty( "columnVec", vectors[1] )
		swapped_image.setProperty( "sliceVec", vectors[2] )
		swapped_image.setProperty( "indexOrigin", index_origin )
		swapped_image.setProperty( "voxelSize", voxel_size )
		if( image.hasProperty("voxelGap") ):
			voxel_gap = image.getProperty("voxelGap")
			voxel_gap[axis1], voxel_gap[axis2] = voxel_gap[axis2], voxel_gap[axis1]
			swapped_image.setProperty("voxelGap", voxel_gap)
	return swapped_image	