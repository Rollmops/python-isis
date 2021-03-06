############################################################
# configure targets
############################################################

set( CORE_FILES util.cpp
				common.cpp
				_application.cpp
				_propmap.cpp
				_convertFromPython.cpp 
				_messageHandler.cpp )


add_library(_util SHARED ${CORE_FILES} )
set_target_properties(_util PROPERTIES PREFIX "")
target_link_libraries( _util ${ISIS_LIB} ${ISIS_LIB_DEPENDS} ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

############################################################
# configure installation
############################################################

set( UTIL_FILES __init__.py )

foreach(pv ${PYTHON_VERSIONS})
execute_process ( COMMAND ${pv} -c "from distutils.sysconfig import get_python_lib; print get_python_lib().lstrip('${PYTHON_PACKAGE_PATH_PREFIX}')" OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE)
set(PYTHON_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")

install(FILES ${UTIL_FILES}
  DESTINATION  ${PYTHON_INSTALL_PATH}/isis/util)

install(TARGETS _util
     LIBRARY DESTINATION  ${PYTHON_INSTALL_PATH}/isis/util
     COMPONENT RuntimeLibraries)
endforeach(pv)
