cmake_minimum_required(VERSION 3.8)
project(buggy_description)

find_package(ament_cmake REQUIRED)

foreach(dir urdf meshes launch)
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
