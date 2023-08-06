find_package(GTest REQUIRED)

macro(add_test_executable name)
  add_executable(${name} ${name}.cpp)
  target_include_directories(${name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src)
  target_link_libraries(${name} GTest::GTest GTest::Main)
  add_test(NAME ${name} COMMAND ${name}) 
endmacro()

file(GLOB TEST_FILE CONFIGURE_DEPENDS *.cpp)

foreach(source ${TEST_FILE})
  get_filename_component(name ${source} NAME_WE)
  add_test_executable(${name})
endforeach()
