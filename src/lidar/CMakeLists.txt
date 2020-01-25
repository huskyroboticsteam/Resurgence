cmake_minimum_required(VERSION 3.2)
project(Rover LANGUAGES CXX)

add_library(PointCloudProcessing 
            PointCloudProcessing.h PointCloudProcessing.cpp)
