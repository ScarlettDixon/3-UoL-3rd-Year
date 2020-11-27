#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define __CL_ENABLE_EXCEPTIONS

#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <ctime>
#include "Assign1.h"

using namespace std;
vects file_read(string filenm) {
	cout << "-------------------------------------------------------------" << endl;
	cerr << "2. Importing Files into code" << endl;
	cerr << "Importing file with name " << filenm << endl;
	const clock_t begin_time = clock();
	ifstream Data_file(filenm);
	int i = 0;
	std::vector<int> temps_int;
	std::vector<float> temps_float;
	std::vector<int> bubb_temps_int;
	std::vector<float> bubb_temps_float;
	std::string weath_Stat_name, year, month, day, time;
	float air_Temp;

	while (Data_file >> weath_Stat_name >> year >> month >> day >> time >> air_Temp)
	{
		temps_float.push_back(air_Temp);
		temps_int.push_back((int)air_Temp);
		bubb_temps_int.push_back((int)air_Temp);
		//bubb vectors used because using local memory requires padding and so median and quartiles would need to be offset or the zeros removed, thought easier to just create a seperate vector without padding
		bubb_temps_float.push_back(air_Temp);
	}
	Data_file.close();
	vects Data = {temps_int, temps_float, bubb_temps_int, bubb_temps_float};
	cout << "Time Taken to import file in seconds: " << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;
	cout << "-------------------------------------------------------------" << endl;
	cerr << "" << endl;
	return Data;

}
/*
		const clock_t import_time = clock(); //Took a few seconds longer than fastest implimentation
		string Data_line;
		while (getline(Data_file, Data_line))
		{

			string Data;
			size_t found = Data_line.find_last_of(" ");
			Data = Data_line.substr(found + 1);
			temps_float.push_back(stof(Data));
			temps_int.push_back(stoi(Data));
			bubb_temps_int.push_back(stoi(Data));
			//bubb vectors used because using local memory requires padding and so median and quartiles would need to be offset or the zeros removed, thought easier to just create a seperate vector without padding
			bubb_temps_float.push_back(stof(Data));
		}
		std::cout << "Time Taken to import file in seconds: " << float(clock() - import_time) / CLOCKS_PER_SEC << endl;
		
		
		/*
		//Incredibly Slow for larger file, takes approx a minute to import
		std::ifstream input("temp_lincolnshire.txt", std::ios_base::in);
		//std::ifstream input("temp_lincolnshire.txt", std::ios_base::in); //Possible options for large dataset: Split data then call kernel twice
		std::string current_Line;
		std::vector<float> temps_float;
		std::vector<int> temps_int;
		const clock_t begin_time = clock();
		while (std::getline(input, current_Line)) {
			std::istringstream currLine(current_Line);
			std::string weath_Stat_name, year, month, day, time;
			float air_Temp;
			if (!(currLine >> weath_Stat_name >> year >> month >> day>> time >> air_Temp)) { break; }
			temps_float.push_back(air_Temp);
			temps_int.push_back((int)air_Temp);
			}
		input.close();
		std::cout << float(clock() - begin_time) / CLOCKS_PER_SEC;
		*/