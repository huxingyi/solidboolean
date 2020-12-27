cmake_minimum_required(VERSION 3.1...3.19)

set(CMAKE_CXX_FLAGS_RELEASE "/O2")

project(test-solidboolean)

include_directories(
    ../thirdparty/eigen
    ../thirdparty/GuigueDevillers03
    ../thirdparty/earcut.hpp/include/mapbox
    ../src
)

add_executable(test-solidboolean 
    main.cpp
    
    ../src/vector3.h
    
    ../src/vector2.h
    
    ../src/double.h
    
    ../src/axisalignedboundingbox.cpp
    ../src/axisalignedboundingbox.h
    
    ../src/axisalignedboundingboxtree.cpp
    ../src/axisalignedboundingboxtree.h
    
    ../src/solidmesh.cpp
    ../src/solidmesh.h
    
    ../src/positionkey.cpp
    ../src/positionkey.h
    
    ../src/retriangulator.cpp
    ../src/retriangulator.h
    
    ../src/solidboolean.cpp
    ../src/solidboolean.h
    
    ../thirdparty/GuigueDevillers03/tri_tri_intersect.c
    ../thirdparty/GuigueDevillers03/tri_tri_intersect.h
    
    ../thirdparty/earcut.hpp/include/mapbox/earcut.hpp
)