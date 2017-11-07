include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(tinyxml)
add_subdirectory(cyCodeBase)
add_subdirectory(lodepng)
add_subdirectory(Catch)

set(SRC
	${tinyxml_SRC}
	${cyCodeBase_SRC}
	${lodepng_SRC}
	${CMAKE_CURRENT_SOURCE_DIR}/materials.h
	${CMAKE_CURRENT_SOURCE_DIR}/materials.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/lights.h
	${CMAKE_CURRENT_SOURCE_DIR}/lights.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/xmlload.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/scene.h
	${CMAKE_CURRENT_SOURCE_DIR}/scene.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/objects.h
	${CMAKE_CURRENT_SOURCE_DIR}/objects.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/viewport.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/render.h
	${CMAKE_CURRENT_SOURCE_DIR}/render.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt
	)

set(prj6_SRC 
	${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
	${SRC}
	PARENT_SCOPE)

set(prj6_tests_SRC
	${CMAKE_CURRENT_SOURCE_DIR}/catch_main.cpp
	${SRC}
	${catch_SRC}
	PARENT_SCOPE)
