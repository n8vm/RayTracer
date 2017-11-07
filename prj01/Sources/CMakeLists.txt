include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(tinyxml)
add_subdirectory(cyCodeBase)
add_subdirectory(Catch)

set(SRC
	${tinyxml_SRC}
	${cyCodeBase_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/xmlload.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/scene.h
	${CMAKE_CURRENT_SOURCE_DIR}/objects.h
	${CMAKE_CURRENT_SOURCE_DIR}/viewport.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/render.h
	${CMAKE_CURRENT_SOURCE_DIR}/IntersectRay.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/render.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	)

set(prj1_SRC 
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${SRC}
	PARENT_SCOPE)

set(prj1_tests_SRC
	${CMAKE_CURRENT_SOURCE_DIR}/catch_main.cpp
	${SRC}
	${catch_SRC}
	PARENT_SCOPE)
