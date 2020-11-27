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

size_t serial_implementation(std::vector<float>temps_float) {
	size_t before_siz = temps_float.size();
	float sum_of_elems = 0;
	float mean_of_elems = 0;
	float std_dev = 0;
	float median = 0;
	float uppermed = 0;
	float lowermed = 0;
	int medlocation = static_cast<int>(before_siz / 2);
	float first_q = 0;
	float third_q = 0;
	const clock_t serial_time = clock();
	for (std::vector<float>::iterator it = temps_float.begin(); it != temps_float.end(); ++it) {
		sum_of_elems += *it;
	}
	std::vector<float> ser_sort_temps_float = temps_float;
	std::sort(ser_sort_temps_float.begin(), ser_sort_temps_float.end());
	mean_of_elems = sum_of_elems / temps_float.size();
	for (std::vector<float>::iterator it = temps_float.begin(); it != temps_float.end(); ++it) {
		std_dev += ((*it - mean_of_elems) * (*it - mean_of_elems));
	}
	std_dev = std_dev / before_siz;
	std_dev = sqrt(std_dev);
	if (before_siz % 2 == 0) {
		//std::cout << medlocation;
		try {
			uppermed = ser_sort_temps_float[medlocation];
			lowermed = ser_sort_temps_float[medlocation - 1];
			median = (uppermed + lowermed) / 2;
		}
		catch (int e) {}
	}
	else {
		median = ser_sort_temps_float[medlocation];


	}
	first_q = ser_sort_temps_float[(medlocation) / 2];
	third_q = ser_sort_temps_float[medlocation + (medlocation / 2)];
	std::cout << "-------------------------------------------------------------" << endl;
	cerr << "3. Serial Float Output - " << endl;
	cerr << "Sum of elements Serial: " << sum_of_elems << endl;
	cerr << "Mean of elements Serial: " << mean_of_elems << endl;
	cerr << "Minimum value Serial: " << ser_sort_temps_float[0] << endl;
	cerr << "Maximum value Serial: " << ser_sort_temps_float[before_siz - 1] << endl;
	cerr << "Standard Deviation Serial: " << std_dev << endl;
	cerr << "1st Quartile Serial : " << first_q << endl;
	cerr << "3rd Quartile Serial : " << third_q << endl;
	cerr << "Median Serial : " << median << endl;
	std::cout << "Time Taken in seconds: " << float(clock() - serial_time) / CLOCKS_PER_SEC << endl;
	std::cout << "-------------------------------------------------------------" << endl;
	cerr << " " << endl;

	return before_siz;
}