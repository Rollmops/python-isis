/*
 * _chunk.hpp
 *
 *  Created on: Oct 21, 2010
 *      Author: tuerke
 */

#ifndef _CHUNK_HPP_
#define _CHUNK_HPP_

#include "DataStorage/chunk.hpp"
#include <boost/python.hpp>
#include "util/_convertToPython.hpp"

using namespace isis::data;

namespace isis
{
namespace python
{
namespace data
{

class _Chunk : public Chunk, boost::python::wrapper<Chunk>
{
public:
	_Chunk ( PyObject *p, const Chunk &base );
	unsigned int typeID_;
private:
	PyObject *self;

};

namespace Chunk {

boost::python::api::object _voxel( const isis::python::data::_Chunk& base, const isis::util::ivector4 &coord );
boost::python::api::object _voxel( const isis::python::data::_Chunk& base, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth );

bool _setVoxel( isis::python::data::_Chunk& base, const isis::util::ivector4 &coord, const api::object &value );
bool _setVoxel( isis::python::data::_Chunk& base, const size_t &first, const size_t &second, const size_t &third, const size_t &fourth, const api::object &value );

bool _convertToType( isis::data::Chunk &base, const unsigned short ID );
bool _convertToType( isis::data::Chunk &base, const unsigned short ID, float scaling, size_t offset );

api::object _getMin( const isis::data::Chunk &base );
api::object _getMax( const isis::data::Chunk &base );
api::object _getMinMax( const isis::data::Chunk &base );

}
}
}
}
#endif /* CHUNK_HPP_ */
