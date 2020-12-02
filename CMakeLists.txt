cmake_minimum_required(VERSION 3.9)

#InitQt("Core;Widgets;LinguistTools;OpenGL")
set(Support_E57 ON CACHE BOOL "Read from E57 format")
if (Support_E57)

set(PUBLIC_HEADERS 
  inc/e57Reader.hpp  
)
set(PROJECT_SRCS
  e57Reader.cpp  
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
set(PUBLIC_HEADER_PATH "${CMAKE_SOURCE_DIR}/e57toPtx/inc")

set(E57Lib "e57reader")
add_library(${E57Lib} STATIC "${PROJECT_SRCS}" "${PUBLIC_HEADERS}")
target_include_directories(${E57Lib} PUBLIC ${PUBLIC_HEADER_PATH})
target_include_directories(${E57Lib} PUBLIC ${CMAKE_SOURCE_DIR})
AddExternalDependency(${E57Lib} PRIVATE Boost)
target_link_libraries(${E57Lib} PUBLIC E57Format)
set_target_properties(${E57Lib} PROPERTIES FOLDER ${E57Lib})
source_group("Source Files" FILES ${PROJECT_SRCS})
source_group("Public Headers" FILES ${PUBLIC_HEADERS})
##############################################################################################

#SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /FORCE")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /DEBUG")

	set(APP "e57toPtx")
	add_definitions(-DSUPPORT_E57)
	add_executable(${APP} e57toPtx.cpp)
	#set_target_properties(${APP} PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
	target_link_libraries(${APP} PRIVATE E57Format)
	target_link_libraries(${APP} PRIVATE ptxlib)
	target_link_libraries(${APP} PRIVATE ${E57Lib})
	AddExternalDependency(${APP} PRIVATE Boost)
	AddExternalDependency(${APP} PRIVATE Eigen)	
	#AddExternalDependency(${APP} PUBLIC glog)
	#AddExternalDependency(${APP} PUBLIC OpenCV)
	set_target_properties(${APP} PROPERTIES FOLDER ${APP})
endif()