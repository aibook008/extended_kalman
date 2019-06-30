project(kalman_cody)

set(sources
    FusionEKF.cpp
    kalman_filter.cpp
    main.cpp
    tools.cpp)
    

add_executable(kalman_cody ${sources})

install(TARGETS kalman_cody RUNTIME DESTINATION bin)
