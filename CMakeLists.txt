cmake_minimum_required(VERSION 3.2)
project(gphyx)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGLM_FORCE_RADIANS -DGLM_PRECISION_MEDIUMP_FLOAT -ffast-math -std=c++14" )

IF (CMAKE_BUILD_TYPE STREQUAL "Release")
    #disable profiling
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNPROFILE -Wall -Werror -Wno-strict-aliasing" )
ENDIF()

IF (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    IF (WIN32)
        # for profiling
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -gdwarf-2" )
    ENDIF()
ENDIF()

IF (WIN32)
    include_directories(c:/DEV/ENV/msys64/mingw64/include/)
    link_directories(c:/DEV/ENV/msys64/mingw64/lib/)
    include_directories(c:/DEV/gamedev/base/include/)
    include_directories(c:/DEV/gamedev/assets/include/)
    include_directories(c:/DEV/gamedev/maths/include/)
    include_directories(c:/DEV/gamedev/SAP/include/)
ELSE()
    include_directories(/home/grynca/DEV/gamedev/base/include/)
    include_directories(/home/grynca/DEV/gamedev/assets/include/)
    include_directories(/home/grynca/DEV/gamedev/maths/include/)
    include_directories(/home/grynca/DEV/gamedev/SAP/include)
ENDIF()

option(USE_SDL2 "Use SDL2" ON)
IF (USE_SDL2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_SDL2=1" )
    IF (WIN32)
        set(LIBS "-lglew32 -lglu32 -lopengl32 -lmingw32 -lSDL2main -lSDL2 -lSDL2_image")
    ELSE()
        set(LIBS "-lGLEW -lGL -lGLU -lSDL2 -lSDL2_image")
    ENDIF()
ENDIF()

include_directories(include/)

set(INC_FILES
        include/gphyx.h
        include/gphyx/gphyx_config.h
        include/gphyx/gphyx_funcs.h
        include/gphyx/gphyx_domain.h
        include/gphyx/Body.h
        include/gphyx/Body.inl
        include/gphyx/Collider.h
        include/gphyx/Collider.inl
        include/gphyx/Speed.h
        include/gphyx/BodiesPairHandlerBase.h
        include/gphyx/PhysicsManagerT.h
        include/gphyx/PhysicsManagerT.inl
        include/gphyx/ManifoldCache.h
        include/gphyx/ManifoldCache.inl
        include/gphyx/joints.h
        include/gphyx/joints/ContactJoint.h
        include/gphyx/joints/ContactJoint.inl
        include/gphyx/joints/RevoluteJoint.h
        include/gphyx/joints/RevoluteJoint.inl
        include/gphyx/joints/Joint.h)
set(SOURCE_FILES
        test/main.cpp
        test/test_dynamics.h
        test/test_collisions.h
        )

add_executable(gphyx ${SOURCE_FILES} ${INC_FILES} )
target_link_libraries(gphyx ${LIBS})
