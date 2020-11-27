
/*
Implementation Data needed to be collected:
 Base -
 min, max and average values, and standard deviation
 Extra - 
 median and its 1st and 3rd quartiles (i.e. 25th and 75th percentiles)

 Evaluation Data needed to be collected:
 Base - 
 memory transfer, kernel execution and total program execution times
 Extra -
 additional optimisation strategies which target the parallel performance of the tool
 execution times for different variants of your algorithm
*/

/*
Patterns and ways to compute data:
map Pattern
stencil pattern
Parallel scan
	Hillis-Steele - Work Efficient
	Blelloch - Span Efficient
Data Reorganisation Patterns
	Gather
	Scatter
Communication
	Barriers and Atomic Functions
	Coalesced Memory Access
Sorting
	Odd Even Sort
	Bitonic Sort
Histogram
	Privatisation
	Performance
*/


//Integer Kernels
kernel void int_reduce_add(global const int* A, global int* B, local int* scratch, global int* C) {
	int id = get_global_id(0);
	int lid = get_local_id(0);
	int gid = get_group_id(0);
	int gsize = get_global_size(0);
	int lsize = get_local_size(0);
	int wnum = get_num_groups(0);

	scratch[lid] = A[id];
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int i = 1; i < lsize; i *= 2) {
		if (!(lid % (i * 2)) && ((lid + i) < lsize))
			scratch[lid] += scratch[lid + i];
		barrier(CLK_LOCAL_MEM_FENCE);
	}

	if (lid == 0) {
		B[gid] = scratch[lid];
		barrier(CLK_LOCAL_MEM_FENCE);
	}
	barrier(CLK_LOCAL_MEM_FENCE);
	if (id == 0) {
		for (int i = 0; i < wnum; i++) { 
			C[0] += B[i];
			barrier(CLK_LOCAL_MEM_FENCE);
		}
	}
}


kernel void int_bubble_even(global int* A) {
	int id = get_global_id(0);
	int N = get_global_size(0);
	int temporary;
	//printf("%i", N);
	//for (int i = 0; i < N; i += 2) {
		if (id % 2 == 1 && id + 1 < N && A[id] > A[id + 1]) {
			temporary = A[id]; A[id] = A[id + 1]; A[id + 1] = temporary;
		}
		barrier(CLK_GLOBAL_MEM_FENCE);	
	//}
}
kernel void int_bubble_odd(global int* A) {
	int id = get_global_id(0);
	int N = get_global_size(0);
	int temporary;
	//printf("%i", N);
	//for (int i = 0; i < N; i += 2) {
		if (id % 2 == 0 && id + 1 < N && A[id] > A[id + 1]) {
			temporary = A[id]; A[id] = A[id + 1]; A[id + 1] = temporary;
		}
		barrier(CLK_GLOBAL_MEM_FENCE);
	//}
}


kernel void int_red_std(global int* A, global int* B, int mean, local int* scratch, global int* C) {
	int id = get_global_id(0);
	int lid = get_local_id(0);
	int gid = get_group_id(0);
	int gsize = get_global_size(0);
	int lsize = get_local_size(0);
	int wnum = get_num_groups(0);

	scratch[lid] = (A[id] - mean) * (A[id] - mean);
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int i = 1; i < lsize; i *= 2) {
		if (!(lid % (i * 2)) && ((lid + i) < lsize))
			scratch[lid] += scratch[lid + i];

		barrier(CLK_LOCAL_MEM_FENCE);
	}
	if (lid == 0) {
		B[gid] = scratch[0];

	}
	barrier(CLK_LOCAL_MEM_FENCE); 
	if (id == 0) {
		for (int i = 0; i < wnum; i++) { 
			C[0] += B[i];
			barrier(CLK_LOCAL_MEM_FENCE);
		}
	}

}



// Float Kernels


kernel void float_reduce_add(global float* A, global float* B, local float* scratch, global float* C) {

	int id = get_global_id(0); //equal to number of global items
	int lid = get_local_id(0); //equal to local items, each value called equal to the number of workgroups totalling global size
	int gid = get_group_id(0); //equal to number of workgroups, called
	int gsize = get_global_size(0);
	int lsize = get_local_size(0);
	int wnum = get_num_groups(0);
	int test = 0;
	//B[id] = A[id];
	//barrier(CLK_GLOBAL_MEM_FENCE);
	
	scratch[lid] = A[id];
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int i = 1; i < lsize; i *= 2) { //Goes through this for every workgroup
		if (!(lid % (i * 2)) && ((lid + i) < lsize))
			scratch[lid] += scratch[lid + i];

		barrier(CLK_LOCAL_MEM_FENCE);
	}
	//by now all values have been added up
	if (lid == 0) {
		B[gid] = scratch[lid];
		/*int test = C[lid] + scratch[lid];
		printf("local = %i, global = %i, group = %i \n", lid, id, gid);
		printf("test = %i \n", test);
		A[id] = scratch[lid];
		printf("test = %i \n", test);
		C[lid] = C[lid] + scratch[lid];*/
		barrier(CLK_LOCAL_MEM_FENCE);
	}
	//barrier(CLK_GLOBAL_MEM_FENCE);//Wait until all locations are stored

	barrier(CLK_LOCAL_MEM_FENCE); 
	
	//if (id == 0) {
		//for (int j = 0; j < wnum; j*=2) { //Adding up the reduced values
			//if (!(gid % (j * 2)) && ((gid + j) < wnum))
			//C[0] += B[j];
		
		//printf("test %i -----------", lid);
	
			//barrier(CLK_LOCAL_MEM_FENCE);
		//}
	//}
	
	if (id == 0) {
		for (int i = 0; i < wnum; i++) { //Adding up the reduced values
			C[0] += B[i];
			barrier(CLK_LOCAL_MEM_FENCE);
		}
	}
	
}

kernel void float_bubble_even(global float* A) {
	int id = get_global_id(0);
	int N = get_global_size(0);
	float temporary;
	if (id % 2 == 1 && id + 1 < N && A[id] > A[id + 1]) {
		temporary = A[id]; A[id] = A[id + 1]; A[id + 1] = temporary;
	}
	barrier(CLK_GLOBAL_MEM_FENCE);
}
kernel void float_bubble_odd(global float* A) {
	int id = get_global_id(0);
	int N = get_global_size(0);
	float temporary;
	if (id % 2 == 0 && id + 1 < N && A[id] > A[id + 1]) {
		temporary = A[id]; A[id] = A[id + 1]; A[id + 1] = temporary;
	}
	barrier(CLK_GLOBAL_MEM_FENCE);
}

kernel void reduce_standard_deviation(global  float* A, global float* B, float mean, local float* scratch, global float* C) {
	int id = get_global_id(0);
	int lid = get_local_id(0);
	int gid = get_group_id(0);
	int gsize = get_global_size(0);
	int lsize = get_local_size(0);
	int wnum = get_num_groups(0);
	//B[id] = A[id];
	//barrier(CLK_GLOBAL_MEM_FENCE);
	scratch[lid] = (A[id] - mean) * (A[id] - mean);
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int i = 1; i < lsize; i *= 2) {
		if (!(lid % (i * 2)) && ((lid + i) < lsize))
			scratch[lid] += scratch[lid + i];

		barrier(CLK_LOCAL_MEM_FENCE);
	}
	if (lid == 0) {
		B[gid] = scratch[0];

	}
	barrier(CLK_LOCAL_MEM_FENCE); //Wait until previous reductions are done
	if (id == 0) {
		for (int i = 0; i < wnum; i++) { //Adding up the reduced values
			C[0] += B[i];
			barrier(CLK_LOCAL_MEM_FENCE);
		}
	}
	
}

	//reduce add the value of each workgroup
	/*
	int lid2 = get_local_id(0);
	scratch[lid2] = B[lid];
	barrier(CLK_LOCAL_MEM_FENCE);

	for (int j = 1; j < lsize; j *= 2) {
		if (!(lid2 % (j * 2)) && ((lid2 + j) < lsize))
			scratch[lid2] += scratch[lid2 + j];

		barrier(CLK_LOCAL_MEM_FENCE);
	}
	C[0] = scratch[0];
	*//*
	for (int j = 0; j < (gsize / lsize); j += 1) {
		//prin
		C[0] += B[i];
		barrier(CLK_LOCAL_MEM_FENCE);
	}
	*/

/*
http://www.bealto.com/gpu-sorting_parallel-bitonic-1.html
kernel void ParallelBitonic_A(global const int * in,global int * out, int inc, int dir)
{
  int i = get_global_id(0); // thread index
  int j = i ^ inc; // sibling to compare

  // Load values at I and J
  int iData = in[i];
  uint iKey = getKey(iData);
  int jData = in[j];
  uint jKey = getKey(jData);

  // Compare
  bool smaller = (jKey < iKey) || ( jKey == iKey && j < i );
  bool swap = smaller ^ (j < i) ^ ((dir & i) != 0);

  // Store
  out[i] = (swap)?jData:iData;
}

//a very simple histogram implementation
kernel void hist_simple(global const int* A, global int* H, global int* nr_bins) {
	int id = get_global_id(0);

	//assumes that H has been initialised to 0
	int bin_index = A[id];//take value as a bin index
	if (id < *nr_bins) {
		atomic_inc(&H[bin_index]);//serial operation, not very efficient!
	}
	else {
		atomic_inc(&H[*nr_bins]);
	}
}

//a double-buffered version of the Hillis-Steele inclusive scan
//requires two additional input arguments which correspond to two local buffers
kernel void scan_add(__global const int* A, global int* B, local int* scratch_1, local int* scratch_2) {
	int id = get_global_id(0);
	int lid = get_local_id(0);
	int N = get_local_size(0);
	local int *scratch_3;//used for buffer swap

	//cache all N values from global memory to local memory
	scratch_1[lid] = A[id];

	barrier(CLK_LOCAL_MEM_FENCE);//wait for all local threads to finish copying from global to local memory

	for (int i = 1; i < N; i *= 2) {
		if (lid >= i)
			scratch_2[lid] = scratch_1[lid] + scratch_1[lid - i];
		else
			scratch_2[lid] = scratch_1[lid];

		barrier(CLK_LOCAL_MEM_FENCE);

		//buffer swap
		scratch_3 = scratch_2;
		scratch_2 = scratch_1;
		scratch_1 = scratch_3;
	}

	//copy the cache to output array
	B[id] = scratch_1[lid];
}
*/