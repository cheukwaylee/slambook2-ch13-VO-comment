#将src中所有的源文件都编译成一个叫做myslam的共享库，
add_library(myslam SHARED
        basicStruct/feature.cpp
        basicStruct/frame.cpp
        basicStruct/mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

#add_library和add_executable生成的可执行文件对象或者库对象都可以作为target_link_libraries的目标

#将${THIRD_PARTY_LIBS}第三方库连接到生成的共享库myslam上
target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
