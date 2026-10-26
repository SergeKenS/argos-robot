cmake_minimum_required(VERSION 3.8)
project(argos_web)

find_package(ament_cmake REQUIRED)

# Rien à compiler : on installe seulement la page web et le fichier de lancement.
install(DIRECTORY web launch DESTINATION share/${PROJECT_NAME})

ament_package()
