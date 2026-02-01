cmake_minimum_required(VERSION 3.16)
project(pendule C)

add_executable(pendule
    main.c
    pendulum.c
    ga.c
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CSFML REQUIRED csfml-graphics csfml-window csfml-system csfml-audio)

target_include_directories(pendule PRIVATE ${CSFML_INCLUDE_DIRS})
target_link_libraries(pendule PRIVATE ${CSFML_LIBRARIES} m)
