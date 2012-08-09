file(GLOB SCI_FILES "*.py")

install(FILES ${SCI_FILES}
  DESTINATION  ${PYTHON_INSTALL_PATH}/isis/scientific)