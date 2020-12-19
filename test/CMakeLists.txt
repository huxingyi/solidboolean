cmake_minimum_required(VERSION 3.1...3.19)

project(test-solidboolean
  VERSION 1.0
  LANGUAGES CXX
)

include_directories(
    ../thirdparty/eigen
    ../src
)

add_executable(test-solidboolean 
    main.cpp
    
    ../src/vector3.h
    
    ../src/vector2.h
    
    ../src/double.h
    
    ../thirdparty/GuigueDevillers03/tri_tri_intersect.c
    ../thirdparty/GuigueDevillers03/tri_tri_intersect.h
    
    ../thirdparty/poly2tri/poly2tri/sweep/advancing_front.cc
    ../thirdparty/poly2tri/poly2tri/sweep/advancing_front.h

    ../thirdparty/poly2tri/poly2tri/sweep/cdt.cc
    ../thirdparty/poly2tri/poly2tri/sweep/cdt.h

    ../thirdparty/poly2tri/poly2tri/sweep/sweep_context.cc
    ../thirdparty/poly2tri/poly2tri/sweep/sweep_context.h

    ../thirdparty/poly2tri/poly2tri/sweep/sweep.cc
    ../thirdparty/poly2tri/poly2tri/sweep/sweep.h

    ../thirdparty/poly2tri/poly2tri/common/shapes.cc
    ../thirdparty/poly2tri/poly2tri/common/shapes.h

    ../thirdparty/poly2tri/poly2tri/common/utils.h

    ../thirdparty/poly2tri/poly2tri/poly2tri.h
)