#include "_image.hpp"
#include "../common.hpp"
#include <numpy/oldnumeric.h>

#include "VoxelOp.hpp"

#ifdef ISIS_PYTHON_MUPARSER_SUPPORT
#include <muParser.h>
#endif
namespace isis
{
namespace python
{
namespace data
{

_Image::_Image ( PyObject *p, const isis::data::Image &base )
	: Image ( base ), boost::python::wrapper< Image >(), self ( p )
{
	updateOrientationMatrices();
}

_Image::_Image ( PyObject *p, const boost::python::numeric::array &array )
	: boost::python::wrapper< Image >(), self( p )
{
	*this = _Image( p, isis::python::data::Image::_createFromArray( array ) );
	updateOrientationMatrices();

}

_Image::_Image ( PyObject *p, const numeric::array &array, const isis::data::Image &image )
	: boost::python::wrapper< Image >(), self( p )
{
	*this = _Image( p, isis::python::data::Image::_createFromArray( array, image ) );
	updateOrientationMatrices();
}

_Image::_Image ( PyObject *p, const numeric::array &array, const isis::util::PropertyMap &map )
	: boost::python::wrapper< Image >(), self( p )
{
	*this = _Image( p, isis::python::data::Image::_createFromArray( array, map ) );
	updateOrientationMatrices();
}

_Image::_Image ( PyObject *p, const Chunk &chunk )
	: boost::python::wrapper< Image >(), self( p )
{
	*this = _Image( p, isis::data::Image( chunk ) );
	updateOrientationMatrices();
}


namespace Image
{

#ifdef ISIS_PYTHON_MUPARSER_SUPPORT
class VoxelOp : public isis::data::VoxelOp<double>
{
	mu::Parser parser;
	double voxBuff;
	isis::util::FixedVector<double, 4> posBuff;
public:
	VoxelOp( std::string expr ) {
		parser.SetExpr( expr );
		parser.DefineVar( std::string( "vox" ), &voxBuff );
		parser.DefineVar( std::string( "pos_x" ), &posBuff[data::rowDim] );
		parser.DefineVar( std::string( "pos_y" ), &posBuff[data::columnDim] );
		parser.DefineVar( std::string( "pos_z" ), &posBuff[data::sliceDim] );
		parser.DefineVar( std::string( "pos_t" ), &posBuff[data::timeDim] );
	}
	bool operator()( double &vox, const isis::util::vector4<size_t>& pos ) {
		voxBuff = vox; //using parser.DefineVar every time would slow down the evaluation
		posBuff = pos;
		vox = parser.Eval();
		return true;
	}

};

bool _applyOperation( isis::data::Image &base, const std::string &operation )
{
	try {
		VoxelOp vop( operation );
		base.foreachVoxel<double>( vop );
	} catch( mu::Parser::exception_type &e ) {
		std::cerr << e.GetMsg() << std::endl;
		return false;
	}

	return true;
}

#endif

api::object _voxel ( const isis::data::Image &base, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth )
{
	const unsigned int typeID = base.getChunk ( first, second, third, fourth, false ).getTypeID();
	return isis::python::data::VoxelOp::getVoxelAsPyObject( base, typeID, first, second, third, fourth );
}

object _voxel ( const isis::data::Image &base, const util::ivector4 &coord )
{
	return _voxel ( base, coord[0], coord[1], coord[2], coord[3] );
}

object _voxelAs ( const isis::data::Image &base, const image_types &type, const isis::util::ivector4 &coord )
{
	return isis::python::data::VoxelOp::getVoxelAsPyObject( base, static_cast<unsigned int>( type ), coord[0], coord[1], coord[2], coord[3] );
}

object _voxelAs ( const isis::data::Image &base, const image_types &type, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth )
{
	return isis::python::data::VoxelOp::getVoxelAsPyObject( base, static_cast<unsigned int>( type ), first, second, third, fourth );
}

object _voxelAs ( const isis::data::Image &base, const int &type, const util::ivector4 &coord )
{
	return _voxelAs( base, static_cast<isis::python::data::image_types>( type ), coord );
}

object _voxelAs ( const isis::data::Image &base, const int &type, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth )
{
	return isis::python::data::VoxelOp::getVoxelAsPyObject( base, type, first, second, third, fourth );
}

bool _setVoxel ( isis::data::Image &base, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth, const api::object &value )
{
	const unsigned int typeID = base.getChunk ( first, second, third, fourth, false ).getTypeID();
	return isis::python::data::VoxelOp::setVoxelAsPyObject( base, typeID, first, second, third, fourth, value );
}

bool _setVoxel ( isis::data::Image &base, const util::ivector4 &coord, const object &value )
{
	return _setVoxel ( base, coord[0], coord[1], coord[2], coord[3], value );
}

bool _setVoxelAs ( isis::data::Image &base, const isis::python::data::image_types &type, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth, const object &value )
{
	return isis::python::data::VoxelOp::setVoxelAsPyObject( base, static_cast<unsigned int>( type ), first, second, third, fourth, value );
}

bool _setVoxelAs ( isis::data::Image &base, const isis::python::data::image_types &type, const util::ivector4 &coord, const object &value )
{
	return isis::python::data::VoxelOp::setVoxelAsPyObject( base, static_cast<unsigned int>( type ), coord[0], coord[1], coord[2], coord[3], value );
}

bool _setVoxelAs ( isis::data::Image &base, const int &type, const util::ivector4 &coord, const object &value )
{
	return _setVoxelAs( base, static_cast<isis::python::data::image_types>( type ), coord, value );
}

bool _setVoxelAs ( isis::data::Image &base, const int &type, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth, const object &value )
{
	return _setVoxelAs( base, static_cast<isis::python::data::image_types>( type ), first, second, third, fourth, value );
}


list _getChunksAsVector ( const isis::data::Image &base )
{
	return isis::python::stdIter2PyList<std::vector<isis::data::Chunk> >( base.copyChunksToVector() );
}

isis::data::Chunk _getChunk ( const isis::data::Image &base, const util::ivector4 &coord, bool copy_metadata )
{
	return base.getChunk ( coord[0], coord[1], coord[2], coord[3], copy_metadata );
}

isis::data::Chunk _getChunkAs ( const isis::data::Image &base, const util::ivector4 &coord, const image_types &type )
{
	return _getChunkAs ( base, coord[0], coord[1], coord[2], coord[3], type );
}

isis::data::Chunk _getChunkAs ( const isis::data::Image &base, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth, const isis::python::data::image_types &type )
{
	isis::data::Chunk ret = base.getChunk ( first, second, third, fourth ); // get a cheap copy
	ret.convertToType ( type );
	return ret;
}

object _getMin ( const isis::data::Image &base )
{
	const util::ValueReference min = base.getMinMax().first;
	return  util::Singletons::get<isis::python::util::_internal::TypesMap, 10>().at (
				min->getTypeID() )->convert ( *min );
}

object _getMax ( const isis::data::Image &base )
{
	const util::ValueReference max = base.getMinMax().second;
	return  util::Singletons::get<isis::python::util::_internal::TypesMap, 10>().at (
				max->getTypeID() )->convert ( *max );
}

object _getMinMax ( const isis::data::Image &base )
{
	const std::pair< isis::util::ValueReference, isis::util::ValueReference> minMax = base.getMinMax();
	object min = isis::util::Singletons::get<isis::python::util::_internal::TypesMap, 10>().at (
					 minMax.first->getTypeID() )->convert ( *minMax.first );
	object max = isis::util::Singletons::get<isis::python::util::_internal::TypesMap, 10>().at (
					 minMax.second->getTypeID() )->convert ( *minMax.second );

	return boost::python::make_tuple( min, max );
}


std::string _getMainOrientationAsString( const isis::data::Image &base )
{
	switch ( base.getMainOrientation() ) {
	case isis::data::Image::sagittal:
		return std::string ( "sagittal" );
		break;
	case isis::data::Image::reversed_sagittal:
		return std::string ( "reversed_sagittal" );
		break;
	case isis::data::Image::axial:
		return std::string ( "axial" );
		break;
	case isis::data::Image::reversed_axial:
		return std::string ( "reversed_axial" );
		break;
	case isis::data::Image::coronal:
		return std::string ( "coronal" );
		break;
	case isis::data::Image::reversed_coronal:
		return std::string ( "reversed_coronal" );
		break;
	default:
		return std::string ( "unknown" );
		break;
	}
}


void _transformCoords ( isis::data::Image &base, boost::python::list matrix, const bool &center )
{
	std::vector< boost::python::list > rows;

	for ( boost::python::ssize_t i = 0; i < boost::python::len ( matrix ); ++i ) {
		rows.push_back ( boost::python::extract< boost::python::list > ( matrix[i] ) );
	}

	boost::numeric::ublas::matrix<float> boostMatrix ( 3, 3 );

	for ( unsigned short i = 0; i < 3; i++ ) {
		for ( unsigned short j = 0; j < 3; j++ ) {
			boostMatrix ( i, j ) = boost::python::extract<float> ( rows[i][j] );
		}
	}

	base.transformCoords ( boostMatrix, center );
}

bool _convertToType ( isis::data::Image &base, image_types type )
{
	return base.convertToType ( static_cast<unsigned short> ( type ) );
}

isis::data::Image _deepCopy( const isis::data::Image &base )
{
	switch ( base.getMajorTypeID() ) {
	case ValueArray<bool>::staticID:
		return MemImage<bool> ( base );
		break;
	case ValueArray<int8_t>::staticID:
		return MemImage<int8_t> ( base );
		break;
	case ValueArray<uint8_t>::staticID:
		return MemImage<uint8_t> ( base );
		break;
	case ValueArray<int16_t>::staticID:
		return MemImage<int16_t> ( base );
		break;
	case ValueArray<uint16_t>::staticID:
		return MemImage<uint16_t> ( base );
		break;
	case ValueArray<int32_t>::staticID:
		return MemImage<int32_t> ( base );
		break;
	case ValueArray<uint32_t>::staticID:
		return MemImage<uint32_t> ( base );
		break;
	case ValueArray<int64_t>::staticID:
		return MemImage<int64_t> ( base );
		break;
	case ValueArray<uint64_t>::staticID:
		return MemImage<uint64_t> ( base );
		break;
	case ValueArray<float>::staticID:
		return MemImage<float> ( base );
		break;
	case ValueArray<double>::staticID:
		return MemImage<double> ( base );
		break;
	case ValueArray<std::complex<float> >::staticID:
		return MemImage<std::complex<float> > ( base );
		break;
	case ValueArray<std::complex<double> >::staticID:
		return MemImage<std::complex<double> > ( base );
		break;
	case ValueArray<isis::util::color24>::staticID:
		return MemImage<isis::util::color24> ( base );
		break;
	case ValueArray<isis::util::color48>::staticID:
		return MemImage<isis::util::color48> ( base );
		break;
	default:
		LOG ( Runtime, error ) << "Unregistered pixel type " << util::getTypeMap() [base.getMajorTypeID()] << ".";
		return MemImage<int8_t> ( base );
	}
}

isis::data::Image _deepCopyAs ( const isis::data::Image &base, image_types type )
{
	isis::data::Image retImage = Image::_deepCopy( base );
	retImage.convertToType ( static_cast<unsigned short> ( type ) );

	return retImage;
}


isis::data::Image _createImage ( image_types type, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth )
{
	switch ( type ) {
	case BOOL:
		return _internal::_internCreateImage<bool> ( first, second, third, fourth ) ;
		break;
	case INT8_T:
		return _internal::_internCreateImage<int8_t> ( first, second, third, fourth ) ;
		break;
	case UINT8_T:
		return _internal::_internCreateImage<uint8_t> ( first, second, third, fourth ) ;
		break;
	case INT16_T:
		return _internal::_internCreateImage<int16_t> ( first, second, third, fourth );
		break;
	case UINT16_T:
		return _internal::_internCreateImage<uint16_t> ( first, second, third, fourth );
		break;
	case INT32_T:
		return _internal::_internCreateImage<int32_t> ( first, second, third, fourth );
		break;
	case UINT32_T:
		return _internal::_internCreateImage<uint32_t> ( first, second, third, fourth );
		break;
	case INT64_T:
		return _internal::_internCreateImage<int64_t> ( first, second, third, fourth );
		break;
	case UINT64_T:
		return _internal::_internCreateImage<uint64_t> ( first, second, third, fourth );
		break;
	case FLOAT:
		return _internal::_internCreateImage<float> ( first, second, third, fourth );
		break;
	case DOUBLE:
		return _internal::_internCreateImage<double> ( first, second, third, fourth );
		break;
	case CFLOAT:
		return _internal::_internCreateImage<std::complex<float> > ( first, second, third, fourth );
		break;
	case CDOUBLE:
		return _internal::_internCreateImage<std::complex<double> > ( first, second, third, fourth );
		break;
	case COLOR_24:
		return _internal::_internCreateImage< isis::util::color24 > ( first, second, third, fourth );
		break;
	case COLOR_48:
		return _internal::_internCreateImage< isis::util::color48 > ( first, second, third, fourth );
		break;

	default:
		LOG ( Runtime, error ) << "Unregistered pixel type ";
		break;
	}

	return isis::data::Image ( isis::data::MemChunk<bool> ( 0, 0, 0, 0 ) );
}

isis::data::Image _cheapCopy ( const isis::data::Image &base )
{
	return base;
}

isis::data::Image _createImageFromChunks ( const list &chunks )
{
	std::list<isis::data::Chunk> chunkList = isis::python::pyList2StdList<isis::data::Chunk>( chunks );
	return isis::data::Image( chunkList );
}

numeric::array _getArray ( _Image &base )
{
	return _getArray( base, static_cast<isis::python::data::image_types>( base.getMajorTypeID() ) );
}


numeric::array _getArray( isis::python::data::_Image &base, isis::python::data::image_types image_type )
{
	import_array()
	const isis::util::ivector4 size = base.getSizeAsVector();
	const size_t relDims = base.getRelevantDims();
	npy_intp dims[relDims];

	for( size_t i = 0; i < relDims; i++ ) {
		dims[i] = size[i];
	}
#define CASE_GET_ARRAY( PTYPE, TYPE, NTYPE ) case PTYPE: { \
	base.makeContiguousChunk<TYPE>(); \
	const boost::python::object obj( boost::python::handle<>( PyArray_SimpleNewFromData( relDims, dims, NTYPE, &base.contiguousChunkList_.back()->voxel<TYPE>( 0 ) ) ) ); \
	return boost::python::extract<boost::python::numeric::array>( obj );\
	break; \
}

	switch( image_type ) {
		CASE_GET_ARRAY( FLOAT, float, NPY_FLOAT );
		CASE_GET_ARRAY( DOUBLE, double, NPY_DOUBLE );
		CASE_GET_ARRAY( INT8_T, int8_t, NPY_INT8 );
		CASE_GET_ARRAY( UINT8_T, uint8_t, NPY_UINT8 );
		CASE_GET_ARRAY( INT16_T, int16_t, NPY_INT16 );
		CASE_GET_ARRAY( UINT16_T, uint16_t, NPY_UINT16 );
		CASE_GET_ARRAY( INT32_T, int32_t, NPY_INT32 );
		CASE_GET_ARRAY( UINT32_T, uint32_t, NPY_UINT32 );
		CASE_GET_ARRAY( INT64_T, int64_t, NPY_INT64 );
		CASE_GET_ARRAY( UINT64_T, uint64_t, NPY_UINT64 );
		CASE_GET_ARRAY( BOOL, bool, NPY_BOOL );
		CASE_GET_ARRAY( CFLOAT, std::complex< float >, NPY_CFLOAT );
		CASE_GET_ARRAY( CDOUBLE, std::complex< double >, NPY_CDOUBLE );
	}
}


isis::data::Image _createFromArray( const boost::python::numeric::array &arr )
{
	import_array();
	const boost::python::object shape = arr.attr( "shape" );
	const boost::python::ssize_t len = boost::python::len( shape );
	util::ivector4 size( 1, 1, 1, 1 );

	for ( boost::python::ssize_t i = 0; i < len; i++ ) {
		size[i] = boost::python::extract<int32_t>( shape[i] );
	}

#define CASE_CREATE_FROM_ARRAY( NTYPE, TYPE ) case NTYPE: {\
	isis::data::MemChunk<TYPE>ch( ( TYPE *) PyArray_DATA( arr.ptr() ), size[3], size[2], size[1], size[0] ); \
	return isis::data::Image( isis::python::data::VoxelOp::getSwappedChunk<TYPE>( ch ) ); \
	break; \
}

	switch( PyArray_TYPE( arr.ptr() ) ) {
		CASE_CREATE_FROM_ARRAY( NPY_FLOAT, float );
		CASE_CREATE_FROM_ARRAY( NPY_DOUBLE, double );
		CASE_CREATE_FROM_ARRAY( NPY_INT8, int8_t );
		CASE_CREATE_FROM_ARRAY( NPY_UINT8, uint8_t );
		CASE_CREATE_FROM_ARRAY( NPY_INT16, int16_t );
		CASE_CREATE_FROM_ARRAY( NPY_UINT16, uint16_t );
		CASE_CREATE_FROM_ARRAY( NPY_INT32, int32_t );
		CASE_CREATE_FROM_ARRAY( NPY_UINT32, uint32_t );
		CASE_CREATE_FROM_ARRAY( NPY_INT64, int64_t );
		CASE_CREATE_FROM_ARRAY( NPY_UINT64, uint64_t );
		CASE_CREATE_FROM_ARRAY( NPY_BOOL, bool );
		CASE_CREATE_FROM_ARRAY( NPY_CFLOAT, std::complex<float> );
		CASE_CREATE_FROM_ARRAY( NPY_CDOUBLE, std::complex<double> );
	default:
		LOG( PythonLog, error ) << "Unregistered datatype: " << PyArray_TYPE( arr.ptr() ) << ". Returning empty image of type double.";
		return _createImage( DOUBLE, size[3], size[2], size[1], size[0] );
	}
}

isis::data::Image _createFromArray ( const numeric::array &arr, const isis::util::PropertyMap &map )
{
	isis::data::Image retImage = _createFromArray( arr );
	retImage.join( map, true );
	return retImage;
}


isis::data::Image _createFromArray ( const numeric::array &arr, const isis::data::Image &image )
{
	isis::data::Image retImage = _createFromArray( arr );
	retImage.join( image, true );
	return retImage;
}

std::string _toString ( const isis::data::Image &base )
{
	std::stringstream output;
	output << "ISIS image of size: " << base.getSizeAsString() << " and type: " << base.getMajorTypeName() << " ( " <<
		   base.copyChunksToVector( false ).size() << " chunk(s) )";
	return output.str();
}


isis::util::PropertyMap _getProperties ( const isis::data::Image &base )
{
	return static_cast<const isis::util::PropertyMap &>( base );
}


} // end namespace Image


}
}
}
