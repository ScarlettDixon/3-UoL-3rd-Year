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

#ifdef __APPLE__
#include <OpenCL/cl.hpp>
#else
#include <CL/cl.hpp>
#endif

#include "Utils.h"
#include "Assign1.h"

void print_help() {
	std::cerr << "Application usage:" << std::endl;

	std::cerr << "  -p : select platform " << std::endl;
	std::cerr << "  -d : select device" << std::endl;
	std::cerr << "  -l : list all platforms and devices" << std::endl;
	std::cerr << "  -h : print this message" << std::endl;
	std::cerr << "  -s : use smaller data set (default larger)" << std::endl;
	std::cerr << "  -s : use sorting on larger dataset" << std::endl;

}





int main(int argc, char **argv) {
	//Part 1 - Command line input ----------------------------------------------------------
	int platform_id = 0;
	int device_id = 0;
	string filenm = "temp_lincolnshire.txt";
	bool small = false;

	for (int i = 1; i < argc; i++)	{
		if ((strcmp(argv[i], "-p") == 0) && (i < (argc - 1))) { platform_id = atoi(argv[++i]); }
		else if ((strcmp(argv[i], "-d") == 0) && (i < (argc - 1))) { device_id = atoi(argv[++i]); }
		else if (strcmp(argv[i], "-l") == 0) { std::cout << ListPlatformsDevices() << std::endl; return 0; }
		else if (strcmp(argv[i], "-h") == 0) { print_help(); return 0;}
		else if (strcmp(argv[i], "-s") == 0) { filenm = "temp_lincolnshire_short.txt"; small = true; }
		else if (strcmp(argv[i], "-b") == 0) { small = true; }
	}

	//detect any potential exceptions
	try {

		//Part 2 - Context and Program Creation ----------------------------------------------------------
		cl::Context context = GetContext(platform_id, device_id);

		//display the selected device
		std::cout << "" << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		std::cout << "1. Device Information -" << endl;
		std::cout << "Running on " << GetPlatformName(platform_id) << ", " << GetDeviceName(platform_id, device_id) << std::endl;

		//create a queue to which we will push commands for the device
		cl::CommandQueue queue(context, CL_QUEUE_PROFILING_ENABLE);

		//2.2 Load & build the device code
		cl::Program::Sources sources;

		AddSources(sources, "kernels.cl");

		cl::Program program(context, sources);

		//build and debug the kernel code
		try {
			program.build();
		}
		catch (const cl::Error& err) {
			std::cout << "Build Status: " << program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			std::cout << "Build Options:\t" << program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			std::cout << "Build Log:\t " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(context.getInfo<CL_CONTEXT_DEVICES>()[0]) << std::endl;
			throw err;
		}

		//Part 3 - Device Information ----------------------------------------------------------
		//Calculate the best possible localsize //Max 
		cl::Device device = context.getInfo<CL_CONTEXT_DEVICES>()[0];
		cerr << "Maximum Device Work Group Size: " << device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() << endl;
		cerr << "Maximum Device Dimensions: " << device.getInfo<CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS>() << endl; //Can I use the extra dimensions to split up the code further
		cerr << "Maximum Work Items Per Workgroup: " << device.getInfo<CL_DEVICE_MAX_WORK_ITEM_SIZES>() << endl;
		cerr << "Global Memory Size: " << device.getInfo <CL_DEVICE_GLOBAL_MEM_SIZE>() << endl;
		cerr << "Local Memory Size: " << device.getInfo <CL_DEVICE_LOCAL_MEM_SIZE>() << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;
		//size_t local_size = 18732;
		size_t local_size = device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE>() / 2; //not using the maximum, as may not work for some devices
		
		//Part 4 - Importing Files ----------------------------------------------------------
		//Quicker than previous implimentation for larger file - approx half the time (23 seconds)
		std::vector<float> temps_float;
		std::vector<float> bubb_temps_float;
		std::vector<int> temps_int;
		std::vector<int> bubb_temps_int;
		vects impor = file_read(filenm); //Goes Through to FileRead.cpp
		temps_float = impor.temps_float;
		bubb_temps_float = impor.bubb_temps_float;
		temps_int = impor.temps_int;
		bubb_temps_int = impor.bubb_temps_int;


		//Part 5 - Serial Implimentation ----------------------------------------------------------
		size_t bfsize = serial_implementation(temps_float);
		
		//Part 6 - Padding-------------------------------------------------------------------------
		//Adding zeros to the vector until the global size will be divisible by the local size 
		size_t temps_padding_size = temps_float.size() % local_size;
		if (temps_padding_size) {
			std::vector<float> t_ext(local_size - temps_padding_size, 0.0f);
			std::vector<int> t_ext2(local_size - temps_padding_size, 0);
			temps_float.insert(temps_float.end(), t_ext.begin(), t_ext.end());
			temps_int.insert(temps_int.end(), t_ext2.begin(), t_ext2.end());
		}
		
		//Part 7 - Parallel Setup------------------------------------------------------------------
		//number of elements 18732 for the short txt 
		//complete these after the padding otherwise kernel will not accept the local value
		size_t temps_float_input_elements = temps_float.size();
		size_t temps_float_input_size = temps_float.size() * sizeof(float); //total size of the input data in bytes
		size_t temps_int_input_size = temps_int.size() * sizeof(int);
		size_t workgroupnum = temps_float_input_elements / local_size;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "4. Parallel Setup - " << endl;
		cerr << "Local size used: " << local_size << endl;
		cerr << "Number of elements before padding: " << bfsize << endl; //small - 18732, big -1873106
		cerr << "Number of elements after padding: " << temps_float_input_elements << endl;
		cerr << "Full size of float elements: " << temps_float_input_size << endl;
		cerr << "Full size of int elements: " << temps_int_input_size << endl;
		cerr << "Number of workgroups that will be created: " << workgroupnum << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;




		//Part 8 - Int reduce add------------------------------------------------------------------
		//std::vector<float> temps_Out(workgroupnum);
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "5. Integer Reduce Add:" << endl;
		std::vector<int> temps_int_Out(workgroupnum); //a vector the size of the number of workgroups
		size_t int_output_size = temps_int_Out.size() * sizeof(int);
		std::vector<int> temps_int_Out2(1);
		size_t int_output_size2 = temps_int_Out2.size() * sizeof(int);

		//Creating Buffers for all input elements
		cl::Buffer int_buffer_In(context, CL_MEM_READ_WRITE, temps_int_input_size);
		cl::Buffer int_buffer_Out(context, CL_MEM_READ_WRITE, int_output_size);
		cl::Buffer int_buffer_Out2(context, CL_MEM_READ_WRITE, int_output_size2);
		
		//Writing the buffers to memory to be accessed by the kernel
		queue.enqueueWriteBuffer(int_buffer_In, CL_TRUE, 0, temps_int_input_size, &temps_int[0]);
		queue.enqueueWriteBuffer(int_buffer_Out, CL_TRUE, 0, int_output_size, &temps_int_Out[0]);
		queue.enqueueWriteBuffer(int_buffer_Out2, CL_TRUE, 0, int_output_size2, &temps_int_Out2[0]);
		
		//Creating the kernel
		cl::Kernel int_reduc_kern = cl::Kernel(program, "int_reduce_add");

		//setting the values to be passed through to the kernel function
		int_reduc_kern.setArg(0, int_buffer_In); int_reduc_kern.setArg(1, int_buffer_Out); 
		int_reduc_kern.setArg(2, cl::Local(local_size * sizeof(int))); int_reduc_kern.setArg(3, int_buffer_Out2);
		
		//creating an event to record data about kernel runtime
		cl::Event int_red_event;

		//Enqueueing th kernel, setting it to run, have set both kernel and localsize
		queue.enqueueNDRangeKernel(int_reduc_kern, cl::NullRange, cl::NDRange(temps_float_input_elements), cl::NDRange(local_size), NULL, &int_red_event);
		
		queue.enqueueReadBuffer(int_buffer_Out, CL_TRUE, 0, int_output_size, &temps_int_Out[0]);
		queue.enqueueReadBuffer(int_buffer_Out2, CL_TRUE, 0, int_output_size2, &temps_int_Out2[0]);
		int int_mean = temps_int_Out2[0] - bfsize;
		cl_ulong int_red_add_time = int_red_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - int_red_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		cerr << "Efficiency of kernel -" << endl;
		std::cout << "Kernel execution time [ns]: " << int_red_add_time << std::endl;
		std::cout << GetFullProfilingInfo(int_red_event, ProfilingResolution::PROF_US) << endl;

		cerr << "Workgroup Information -" << endl;
		cerr << "Preferred Work Group Size Multiple: " << int_reduc_kern.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device) << endl;
		//cerr << "Work group size used: " << int_reduc_kern.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << endl;
		cerr << "Amount of local memory used by kernel: " << int_reduc_kern.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device) << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << " " << endl;



		//Part 9 -float reduce add-----------------------------------------------------------------
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "6. Float Reduce Add:" << endl;
		std::vector<float> temps_float_Out(workgroupnum);
		std::vector<float> temps_float_Tot(1);
		size_t output_size = temps_float_Out.size() * sizeof(float);
		size_t output_size2 = temps_float_Tot.size() * sizeof(float);
		//cerr << "Creating Buffers...";
		cl::Buffer buffer_In(context, CL_MEM_READ_WRITE, temps_float_input_size);
		cl::Buffer buffer_Out(context, CL_MEM_READ_WRITE, output_size);
		cl::Buffer buffer_Tot(context, CL_MEM_READ_WRITE, output_size2);
		//cerr << "Done" << endl;

		//cerr << "Copying to device memory...";
		queue.enqueueWriteBuffer(buffer_In, CL_TRUE, 0, temps_float_input_size, &temps_float[0]);
		queue.enqueueWriteBuffer(buffer_Out, CL_TRUE, 0, output_size, &temps_float_Out[0]);
		queue.enqueueWriteBuffer(buffer_Tot, CL_TRUE, 0, output_size2, &temps_float_Tot[0]);
		//queue.enqueueWriteBuffer(buffer_Out2, CL_TRUE, 0, output2_size, &temps_Out[0]);
		//queue.enqueueWriteBuffer(buffer_In, CL_TRUE, 0, input_size, &test[0]);
		//cerr << "Done" << endl;

		//cerr << "Creating kernel and setting arg...";
		cl::Kernel kernel = cl::Kernel(program, "float_reduce_add");
		kernel.setArg(0, buffer_In); kernel.setArg(1, buffer_Out); kernel.setArg(2, cl::Local(local_size * sizeof(float))); kernel.setArg(3, buffer_Tot);
		//kernel.setArg(2, buffer_Out2);
		//cerr << "Done" << endl;

		cl::Event prof_event;

		//cerr << "Enqueueing Kernel...";
		queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(temps_float_input_elements), cl::NDRange(local_size), NULL, &prof_event);
		//cerr << "Done" << endl;

		//cerr << "Reading Buffers...";
		queue.enqueueReadBuffer(buffer_Out, CL_TRUE, 0, output_size, &temps_float_Out[0]);
		queue.enqueueReadBuffer(buffer_Tot, CL_TRUE, 0, output_size2, &temps_float_Tot[0]);
		//cerr << "Done" << endl;
		float float_mean = temps_float_Tot[0] / bfsize;
		cl_ulong float_red_add_time = prof_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - prof_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		cerr << "Efficiency of kernel -" << endl;
		std::cout << "Kernel execution time [ns]: " << float_red_add_time << std::endl;
		std::cout << GetFullProfilingInfo(prof_event, ProfilingResolution::PROF_US) << endl;

		cerr << "Workgroup Information -" << endl;
		cerr << "Preferred Work Group Size Multiple: " << kernel.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device) << endl;
		//cerr << "Work group size used: " << kernel.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << endl;
		cerr << "Amount of local memory used by kernel: " << kernel.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device) << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;



		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "7. Integer Reduce Standard Deviation:" << endl;
		std::vector<int> temps_int_Out_std(workgroupnum);
		std::vector<int> temps_int_Tot_std(1);
		size_t output_size_std_i = temps_int_Out_std.size() * sizeof(int);
		size_t output_size2_std_i = temps_int_Tot_std.size() * sizeof(int);
		//cerr << "Creating Buffers...";
		cl::Buffer buffer_In_std_i(context, CL_MEM_READ_WRITE, temps_int_input_size);
		cl::Buffer buffer_Out_std_i(context, CL_MEM_READ_WRITE, output_size_std_i);
		cl::Buffer buffer_Tot_std_i(context, CL_MEM_READ_WRITE, output_size2_std_i);
		//cerr << "Done" << endl;

		//cerr << "Copying to device memory...";
		queue.enqueueWriteBuffer(buffer_In_std_i, CL_TRUE, 0, temps_int_input_size, &temps_int[0]);
		queue.enqueueWriteBuffer(buffer_Out_std_i, CL_TRUE, 0, output_size_std_i, &temps_int_Out_std[0]);
		queue.enqueueWriteBuffer(buffer_Tot_std_i, CL_TRUE, 0, output_size2_std_i, &temps_int_Tot_std[0]);
		//queue.enqueueWriteBuffer(buffer_Out2, CL_TRUE, 0, output2_size, &temps_Out[0]);
		//queue.enqueueWriteBuffer(buffer_In, CL_TRUE, 0, input_size, &test[0]);
		//cerr << "Done" << endl;

		//cerr << "Creating kernel and setting arg...";
		cl::Kernel std_kern_i = cl::Kernel(program, "int_red_std");
		std_kern_i.setArg(0, buffer_In_std_i);
		std_kern_i.setArg(1, buffer_Out_std_i);
		std_kern_i.setArg(2, int_mean);
		std_kern_i.setArg(3, cl::Local(local_size * sizeof(int)));
		std_kern_i.setArg(4, buffer_Tot_std_i);
		//kernel.setArg(2, buffer_Out2);
		//cerr << "Done" << endl;

		cl::Event std_event_i;

		//cerr << "Enqueueing Kernel...";
		queue.enqueueNDRangeKernel(std_kern_i, cl::NullRange, cl::NDRange(temps_float_input_elements), cl::NDRange(local_size), NULL, &std_event_i);
		//cerr << "Done" << endl;

		//cerr << "Reading Buffers...";
		queue.enqueueReadBuffer(buffer_Out_std_i, CL_TRUE, 0, output_size_std_i, &temps_int_Out_std[0]);
		queue.enqueueReadBuffer(buffer_Tot_std_i, CL_TRUE, 0, output_size2_std_i, &temps_int_Tot_std[0]);
		//cerr << "Done" << endl;
		cl_ulong int_reduce_std = std_event_i.getProfilingInfo<CL_PROFILING_COMMAND_END>() - std_event_i.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		cerr << "Efficiency of kernel -" << endl;
		std::cout << "Kernel execution time [ns]: " << int_reduce_std << std::endl;
		std::cout << GetFullProfilingInfo(std_event_i, ProfilingResolution::PROF_US) << endl;

		cerr << "Workgroup Information -" << endl;
		cerr << "Preferred Work Group Size Multiple: " << std_kern_i.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device) << endl;
		//cerr << "Work group size used: " << std_kern_i.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << endl;
		cerr << "Amount of local memory used by kernel: " << std_kern_i.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device) << endl;
		//cerr << temps_int_Out_std << endl;
		//cerr << temps_float_Tot_std << endl;

		int remove_padding = temps_padding_size * (0 - int_mean) * (0 - int_mean);
		//cerr << temps_padding_size << std::endl;

		int std_dev_i = temps_int_Tot_std[0] - remove_padding;
		//cerr << remove_padding << "   " << std_dev_f << endl;
		//int test = std_dev_f - remove_padding;
		//cerr << test << endl;
		std_dev_i = std_dev_i / temps_int.size();
		std_dev_i = sqrt(std_dev_i);
		//cerr << std_dev_f << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;


		//Part 10 -float reduce stddev-----------------------------------------------------------------
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "8. Float Reduce Standard Deviation:" << endl;
		std::vector<float> temps_float_Out_std(workgroupnum);
		std::vector<float> temps_float_Tot_std(1);
		size_t output_size_std = temps_float_Out_std.size() * sizeof(float);
		size_t output_size2_std = temps_float_Tot_std.size() * sizeof(float);
		//cerr << "Creating Buffers...";
		cl::Buffer buffer_In_std(context, CL_MEM_READ_WRITE, temps_float_input_size);
		cl::Buffer buffer_Out_std(context, CL_MEM_READ_WRITE, output_size_std);
		cl::Buffer buffer_Tot_std(context, CL_MEM_READ_WRITE, output_size2_std);
		//cerr << "Done" << endl;

		//cerr << "Copying to device memory...";
		queue.enqueueWriteBuffer(buffer_In_std, CL_TRUE, 0, temps_float_input_size, &temps_float[0]);
		queue.enqueueWriteBuffer(buffer_Out_std, CL_TRUE, 0, output_size_std, &temps_float_Out_std[0]);
		queue.enqueueWriteBuffer(buffer_Tot_std, CL_TRUE, 0, output_size2_std, &temps_float_Tot_std[0]);
		//queue.enqueueWriteBuffer(buffer_Out2, CL_TRUE, 0, output2_size, &temps_Out[0]);
		//queue.enqueueWriteBuffer(buffer_In, CL_TRUE, 0, input_size, &test[0]);
		//cerr << "Done" << endl;

		//cerr << "Creating kernel and setting arg...";
		cl::Kernel std_kern_f = cl::Kernel(program, "reduce_standard_deviation");
		std_kern_f.setArg(0, buffer_In_std); 
		std_kern_f.setArg(1, buffer_Out_std);
		std_kern_f.setArg(2, float_mean);
		std_kern_f.setArg(3, cl::Local(local_size * sizeof(float)));
		std_kern_f.setArg(4, buffer_Tot_std);
		//kernel.setArg(2, buffer_Out2);
		//cerr << "Done" << endl;

		cl::Event std_event_f;

		//cerr << "Enqueueing Kernel...";
		queue.enqueueNDRangeKernel(std_kern_f, cl::NullRange, cl::NDRange(temps_float_input_elements), cl::NDRange(local_size), NULL, &std_event_f);
		//cerr << "Done" << endl;

		//cerr << "Reading Buffers...";
		queue.enqueueReadBuffer(buffer_Out_std, CL_TRUE, 0, output_size_std, &temps_float_Out_std[0]);
		queue.enqueueReadBuffer(buffer_Tot_std, CL_TRUE, 0, output_size2_std, &temps_float_Tot_std[0]);
		//cerr << "Done" << endl;
		cl_ulong float_reduce_std = std_event_f.getProfilingInfo<CL_PROFILING_COMMAND_END>() - std_event_f.getProfilingInfo<CL_PROFILING_COMMAND_START>();
		cerr << "Efficiency of kernel -" << endl;
		std::cout << "Kernel execution time [ns]: " << float_reduce_std << std::endl;
		std::cout << GetFullProfilingInfo(std_event_f, ProfilingResolution::PROF_US) << endl;

		cerr << "Workgroup Information -" << endl;
		cerr << "Preferred Work Group Size Multiple: " << std_kern_f.getWorkGroupInfo<CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE>(device) << endl;
		//cerr << "Work group size used: " << std_kern_f.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << endl;
		cerr << "Amount of local memory used by kernel: " << std_kern_f.getWorkGroupInfo<CL_KERNEL_LOCAL_MEM_SIZE>(device) << endl;
		//cerr << temps_float_Out_std << endl;
		//cerr << temps_float_Tot_std << endl;

		float remove_padding_f = temps_padding_size * (0.0f - float_mean) * (0.0f - float_mean);
		//cerr << temps_padding_size << std::endl;
		
		float std_dev_f = temps_float_Tot_std[0] - remove_padding_f;
		//cerr << remove_padding << "   " << std_dev_f << endl;
		//float test = std_dev_f - remove_padding;
		//cerr << test << endl;
		std_dev_f  = std_dev_f / temps_float.size();
		std_dev_f = sqrt(std_dev_f);
		//cerr << std_dev_f << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;



		std::cout << "-------------------------------------------------------------" << endl;
		int par_min = 0;
		int par_max = 0;
		int par_medlocation = 0;
		int par_med = 0;
		int par_uppermed = 0;
		int par_lowermed = 0;
		int par_first_q = 0;
		int par_third_q = 0;
		float par_min_f = 0.0f;
		float par_max_f = 0.0f;
		int par_medlocation_f = 0;
		float par_med_f = 0.0f;
		float par_uppermed_f = 0.0f;
		float par_lowermed_f = 0.0f;
		float par_first_q_f = 0.0f;
		float par_third_q_f = 0.0f;
		cl_ulong int_kernel_time = 0;
		cl_ulong float_kernel_time = 0;
		if (small == true) {
			//Part 11 - int parallel bubble sort ------------------------------------------------------------------
			cerr << "9. Integer odd-even sort:" << endl;
			cl::Kernel int_bubble_e = cl::Kernel(program, "int_bubble_even");
			cl::Kernel int_bubble_o = cl::Kernel(program, "int_bubble_odd");
			size_t bub_int_temp_size = bubb_temps_int.size() * sizeof(int);
			cl::Buffer bub_buffer_In(context, CL_MEM_READ_WRITE, bub_int_temp_size);
			queue.enqueueWriteBuffer(bub_buffer_In, CL_TRUE, 0, bub_int_temp_size, &bubb_temps_int[0]);

			cl::Event bub_event;
			//Due to a for loop and continued initialising of kernels a time value will be used instead
			//cl_ulong total_time = 0;
			const clock_t int_bub_time = clock();
			for (int i = 0; i < (bubb_temps_int.size() / 2); i++) {
				int_bubble_e.setArg(0, bub_buffer_In);
				queue.enqueueNDRangeKernel(int_bubble_e, cl::NullRange, cl::NDRange(bubb_temps_int.size()), cl::NullRange, NULL, &bub_event);
				//total_time += bub_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - bub_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
				int_bubble_o.setArg(0, bub_buffer_In);
				queue.enqueueNDRangeKernel(int_bubble_o, cl::NullRange, cl::NDRange(bubb_temps_int.size()), cl::NullRange, NULL, &bub_event);
				//total_time = total_time + bub_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - bub_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
			}
			cerr << "Time taken for whole execution in seconds: "<< float(clock() - int_bub_time) / CLOCKS_PER_SEC << endl;
			queue.enqueueReadBuffer(bub_buffer_In, CL_TRUE, 0, bub_int_temp_size, &bubb_temps_int[0]);
			//cout << bubb_temps_int;
			//std::cout << bubb_temps_int;
			par_min = bubb_temps_int[0];
			par_max = bubb_temps_int[(bubb_temps_int.size() - 1)];
			par_medlocation = (bubb_temps_int.size() / 2);
			par_med = 0;
			par_uppermed = 0;
			par_lowermed = 0;
			if (bubb_temps_int.size() % 2 == 0) {
				//std::cout << medlocation;
				par_uppermed = bubb_temps_int[par_medlocation];
				par_lowermed = bubb_temps_int[par_medlocation - 1];
				par_med = (par_uppermed + par_lowermed) / 2;
			}
			else {
				par_med = bubb_temps_int[par_medlocation];


			}
			int_kernel_time = bub_event.getProfilingInfo<CL_PROFILING_COMMAND_END>() - bub_event.getProfilingInfo<CL_PROFILING_COMMAND_START>();
			par_first_q = bubb_temps_int[(par_medlocation) / 2];
			par_third_q = bubb_temps_int[par_medlocation + (par_medlocation / 2)];
			cerr << "Efficiency of kernel -" << endl;
			std::cout << "Single Kernel execution time [ns]: " << int_kernel_time << std::endl;
			cerr << "Number of kernels called: " << bubb_temps_int.size() << endl;
			int_kernel_time = bubb_temps_int.size() * int_kernel_time;
			cerr << "Approximate execution time for all kernels [ns]: " << int_kernel_time << endl;
			//std::cout << "Kernel execution time [ns]: " << total_time << std::endl;
			std::cout << GetFullProfilingInfo(bub_event, ProfilingResolution::PROF_US) << endl;
			std::cout << "-------------------------------------------------------------" << endl;
			cerr << "" << endl;


			//Part 12 - float parallel bubble sort-------------------------------------------------------------------------
			std::cout << "-------------------------------------------------------------" << endl;
			cerr << "10. Float odd-even sort:" << endl;
			cl::Kernel float_bubble_e = cl::Kernel(program, "float_bubble_even");
			cl::Kernel float_bubble_o = cl::Kernel(program, "float_bubble_odd");
			size_t bub_float_temp_size = bubb_temps_float.size() * sizeof(int);
			cl::Buffer bub_buffer_In_f(context, CL_MEM_READ_WRITE, bub_int_temp_size);
			queue.enqueueWriteBuffer(bub_buffer_In_f, CL_TRUE, 0, bub_int_temp_size, &bubb_temps_float[0]);

			cl::Event bub_event_f;
			const clock_t float_bub_time = clock();
			for (int i = 0; i < (bubb_temps_float.size() / 2); i++) {
				float_bubble_e.setArg(0, bub_buffer_In_f);
				queue.enqueueNDRangeKernel(float_bubble_e, cl::NullRange, cl::NDRange(bubb_temps_float.size()), cl::NullRange, NULL, &bub_event_f);
				float_bubble_o.setArg(0, bub_buffer_In_f);
				queue.enqueueNDRangeKernel(float_bubble_o, cl::NullRange, cl::NDRange(bubb_temps_float.size()), cl::NullRange, NULL, &bub_event_f);
			}
			cerr << "Time taken for whole execution in seconds: " << float(clock() - float_bub_time) / CLOCKS_PER_SEC << endl;
			queue.enqueueReadBuffer(bub_buffer_In_f, CL_TRUE, 0, bub_float_temp_size, &bubb_temps_float[0]);
			//std::cout << bubb_temps_float;
			par_min_f = bubb_temps_float[0];
			par_max_f = bubb_temps_float[(bubb_temps_float.size() - 1)];
			par_medlocation_f = static_cast<int>(bubb_temps_float.size() / 2);
			if (bubb_temps_float.size() % 2 == 0) {
				//std::cout << medlocation;
				par_uppermed_f = bubb_temps_float[par_medlocation_f];
				par_lowermed_f = bubb_temps_float[par_medlocation_f - 1];
				par_med_f = (par_uppermed_f + par_lowermed_f) / 2;
			}
			else {
				par_med_f = bubb_temps_float[par_medlocation_f];


			}
			float_kernel_time = bub_event_f.getProfilingInfo<CL_PROFILING_COMMAND_END>() - bub_event_f.getProfilingInfo<CL_PROFILING_COMMAND_START>();
	
			par_first_q_f = bubb_temps_float[(par_medlocation_f) / 2];
			par_third_q_f = bubb_temps_float[par_medlocation_f + (par_medlocation_f / 2)];
			cerr << "Efficiency of kernel -" << endl;
			std::cout << "Single Kernel execution time [ns]: " << float_kernel_time<< std::endl;
			cerr << "Number of kernels called: "  << bubb_temps_float.size() << endl;
			float_kernel_time = bubb_temps_float.size() * float_kernel_time;
			cerr << "Approximate execution time for all kernels [ns]: " <<  float_kernel_time <<  endl;
			std::cout << GetFullProfilingInfo(bub_event_f, ProfilingResolution::PROF_US) << endl;
			std::cout << "-------------------------------------------------------------" << endl;
			cerr << "" << endl;
		}
		cerr << "" << endl;
		
		//float sum_of_elems2 = 0;
		//for (std::vector<float>::iterator it2 = temps_float_Out.begin(); it2 != temps_float_Out.end(); ++it2) {
			//sum_of_elems2 += *it2;
		//}
		
		//std::cout << "All Output elements Parallel = " << temps_Out << std::endl;
		//std::cout << "Out = " << temps_Out[local_size + 1] << std::endl;
		//std::cout << "Number of Elements = " << temps_input_elements << std::endl;
		//std::cout << "C = " << C << std::endl;
		//int num;
		//clGetDeviceInfo(device_id, );
		//cerr << "Max Work Group Size: " << device.getInfo<CL_DEVICE_MAX_WORK_GROUP_SIZE> << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "Final Parallel Output -" << endl;
		//cerr << "Integer Reduce Add Sum: " << temps_int_Out2[0] << std::endl;
		cerr << "Integer Reduce Add Mean: " << temps_int_Out2[0] / bfsize << std::endl;
		//cerr << "Integer Reduce Add Standard deviation : " << std_dev_i << std::endl;
		if (small == true) {
			cerr << "Integer Odd-Even Minimum: " << par_min << std::endl;
			cerr << "Integer Odd-Even Maximum: " << par_max << std::endl;
			cerr << "Integer Odd-Even Median: " << par_med << std::endl;
			cerr << "Integer Odd-Even 1st Quartile: " << par_first_q << std::endl;
			cerr << "Integer Odd-Even 3rd Quartile: " << par_third_q << std::endl;
		}
		//cerr << temps_int << std::endl;
		cerr << "" << endl;
		
		//cerr << "Float Reduce Add Sum:" << temps_float_Out << std::endl;
		cerr << "Float Reduce Add Sum: " << temps_float_Tot[0] << std::endl;
		cerr << "Float Reduce Add Mean: " << temps_float_Tot[0] / bfsize << std::endl;
		cerr << "Float Reduce Add Standard deviation : " << std_dev_f << std::endl;
		if (small == true) {
			cerr << "Float Odd-Even Minimum: " << par_min_f << std::endl;
			cerr << "Float Odd-Even Maximum: " << par_max_f << std::endl;
			cerr << "Float Odd-Even Median: " << par_med_f << std::endl;
			cerr << "Float Odd-Even 1st Quartile: " << par_first_q_f << std::endl;
			cerr << "Float Odd-Even 3rd Quartile: " << par_third_q_f << std::endl;
		}
		cerr << "" << endl;
		cl_ulong total_kernel_time = int_red_add_time + float_red_add_time + int_reduce_std + float_reduce_std + int_kernel_time + float_kernel_time;
		cerr << "Total Parallel Kernel Time [ns]: " << total_kernel_time << std::endl;
		float tot_sec = (float)total_kernel_time / 1000000000;
		cerr << "Total Parallel Kernel Time in seconds: " << tot_sec << std::endl;
		std::cout << "-------------------------------------------------------------" << endl;
		std::cout << "-------------------------------------------------------------" << endl;
		cerr << "" << endl;

		


	}
	catch (cl::Error err) {
		std::cerr << "ERROR: " << err.what() << ", " << getErrorString(err.err()) << std::endl;
	}
	
	return 0;
}



/*
Thoughts about Efficiency:
Reading in of file is not as efficient as it can be, reading entire line every time and checking each value slows down larger file read
Reduce Add is fairly efficient but other addition implimentations might be quicker
Because of the odd-even sort having such large big O it can take minutes to complete the large data set, bitonic or quicksort 
No massive difference in float and int although int seems to take longer for the reduce add to calculate mean
*/

/*References: 
https://stackoverflow.com/questions/7868936/read-file-line-by-line-using-ifstream-in-c
http://www.bealto.com
Lincoln university lecture slides
*/


/*
		//Testing Area
		std::vector<int> test_I = { -1, -2, -17, 1,8,6,4,7};
		size_t test_local_size = 5;
		size_t test_I_input_elements = test_I.size();
		size_t test_I_input_size = test_I.size() * sizeof(int);
		cl::Buffer buffer_test(context, CL_MEM_READ_WRITE, test_I_input_size);
		queue.enqueueWriteBuffer(buffer_test, CL_TRUE, 0, test_I_input_size, &test_I[0]);

		std::vector<int> out_I(test_I_input_elements);
		size_t out_I_input_elements = out_I.size();
		size_t out_I_input_size = out_I.size() * sizeof(int);
		cl::Buffer out_test(context, CL_MEM_READ_WRITE, out_I_input_size);
		queue.enqueueWriteBuffer(out_test, CL_TRUE, 0, out_I_input_size, &out_I[0]);

		//cl::Kernel test_kernel1 = cl::Kernel(program, "int_bubble_sort");
		//test_kernel1.setArg(0, buffer_test);
		for (int i = 0; i < (test_I.size() / 2); i++) {
			int_bubble_e.setArg(0, bub_buffer_In);
			queue.enqueueNDRangeKernel(int_bubble_e, cl::NullRange, cl::NDRange(test_I.size()), cl::NullRange);
			int_bubble_o.setArg(0, bub_buffer_In);
			queue.enqueueNDRangeKernel(int_bubble_o, cl::NullRange, cl::NDRange(test_I.size()), cl::NullRange);
		}
		//cl::Kernel test_kernel2 = cl::Kernel(program, "int_reduce_add");
		//test_kernel2.setArg(0, buffer_test); test_kernel2.setArg(1, out_test); test_kernel2.setArg(2, cl::Local(local_size * sizeof(float)));

		//cl::Event test_event1;
		//cl::Event test_event2;

		//queue.enqueueNDRangeKernel(test_kernel1, cl::NullRange, cl::NDRange(test_I_input_elements), cl::NullRange, NULL, &test_event1);
		queue.enqueueReadBuffer(buffer_test, CL_TRUE, 0, test_I_input_size, &test_I[0]);
		//queue.enqueueNDRangeKernel(test_kernel2, cl::NullRange, cl::NDRange(test_I_input_elements), cl::NullRange, NULL, &test_event2);
		//queue.enqueueReadBuffer(out_test, CL_TRUE, 0, out_I_input_size, &out_I[0]);

		std::cout << "Integer Bubble Output = " << test_I << std::endl;
		std::cout << "Integer Reduction Output = " << out_I << std::endl;
		*/
		/*
int int_reduce_add(std::vector<int>padded_temps, size_t local_size) {
	size_t temps_int_inp_size = padded_temps.size() * sizeof(int);
	return 0;
}
int int_bubble_sort() {
	return 0;
}
float float_reduce_add(std::vector<float>padded_temps, size_t local_size) {

}
*/