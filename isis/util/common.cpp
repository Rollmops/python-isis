#include "common.hpp"
#include "../common.hpp"
#include <DataStorage/common.hpp>

namespace isis
{
namespace python
{
void _setPythonLogger ( const boost::python::api::object& mH, LogLevel level )
{
	isis::python::enableLogGlobal<isis::python::util::PythonMessageHandler>( level );
	isis::python::enableLogGlobalDebug<isis::python::util::PythonMessageHandlerDebug>( level );
	isis::python::util::PythonMessageHandler::pythonLogger_ = mH;
}

	
void _setPythonLogger ( const boost::python::api::object &mH )
{
	_setPythonLogger( mH, isis::verbose_info );
}
boost::python::api::object _getPythonLogger()
{
	if( isis::python::util::PythonMessageHandler::pythonLogger_ ) {
		return isis::python::util::PythonMessageHandler::pythonLogger_;
	} else {
		LOG( PythonLog, warning ) << "No python logger found. Use \"setLogger()\" to set the python logger";
		return boost::python::api::object();
	}
}

bool _hasLogger()
{
	return isis::python::util::PythonMessageHandler::pythonLogger_;
}



}
}