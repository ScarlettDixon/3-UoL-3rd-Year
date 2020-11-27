#pragma once 
#include <iostream>
#include <vector>
#include <sstream>
#include <string>


//Found within FileRead.cpp
struct vects
{
	std::vector<int> temps_int;
	std::vector<float>temps_float;
	std::vector<int> bubb_temps_int;
	std::vector<float> bubb_temps_float;
	

};
vects file_read(std::string filenm);

//Found within Serial.cpp
size_t serial_implementation(std::vector<float>temps_float);