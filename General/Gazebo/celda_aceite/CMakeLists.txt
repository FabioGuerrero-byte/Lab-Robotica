cmake_minimum_required(VERSION 3.8)
project(celda_aceite)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config worlds models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/coordinador.py scripts/monitor_choques.py scripts/interfaz_r3.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
