file(GLOB SRC_FILE CONFIGURE_DEPENDS *.cpp *.h) 

add_executable(nespp 
  ${SRC_FILE}
)
