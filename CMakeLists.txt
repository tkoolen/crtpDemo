cmake_minimum_required(VERSION 2.8.6)
project(eigenAutoDiffPolynomial-superbuild)
include(ExternalProject)

ExternalProject_Add(eigen
  PREFIX eigen
  URL https://bitbucket.org/eigen/eigen/get/3.2.5.tar.bz2
  URL_HASH MD5=21a928f6e0f1c7f24b6f63ff823593f5
  CMAKE_ARGS -D CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

ExternalProject_Add(crtpDemo
  PREFIX eigenAutoDiffPolynomial
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/src
  CMAKE_ARGS -D CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  DEPENDS eigen)
  
