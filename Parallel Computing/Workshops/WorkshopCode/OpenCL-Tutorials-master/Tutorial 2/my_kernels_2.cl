//a simple OpenCL kernel which copies all pixels from A to B
kernel void identity(global const uchar4* A, global uchar4* B) {
	int id = get_global_id(0);
	//printf("work item id = %d\n", id);
	B[id] = A[id];
	uchar4 pixel;
	pixel.y = 255; //set the maximum value for the second field of uchar4 -//green channel
}

//simple 2D identity kernel
kernel void identity2D(global const uchar4* A, global uchar4* B) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int width = get_global_size(0); //width in pixels
	int id = x + y*width;
	B[id] = A[id];
}

//2D averaging filter
kernel void avg_filter2D(global const uchar4* A, global uchar4* B) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int width = get_global_size(0); //width in pixels
	int id = x + y*width;

	uint4 result = (uint4)(0);//zero all 4 components

	for (int i = (x-1); i <= (x+1); i++)
	for (int j = (y-1); j <= (y+1); j++) 
		result += convert_uint4(A[i + j*width]); //convert pixel values to uint4 so the sum can be larger than 255

	//result /= (uint4)(9); //normalise all components (the result is a sum of 9 values) 
	result /= (uint4)(3); //normalise all components (the result is a sum of 9 values), use three to stop darkening, find out why
	B[id] = convert_uchar4(result); //convert back to uchar4 
}

//2D 3x3 convolution kernel
kernel void convolution2D(global const uchar4* A, global uchar4* B, constant float* mask) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int width = get_global_size(0); //width in pixels
	int id = x + y*width;

	float4 result = (float4)(0.0f,0.0f,0.0f,0.0f);//zero all 4 components

	for (int i = (x-1); i <= (x+1); i++)
	for (int j = (y-1); j <= (y+1); j++)
		result += convert_float4(A[i + j*width])*(float4)(mask[i-(x-1) + (j-(y-1))*3]);//convert pixel and mask values to float4

	B[id] = convert_uchar4(3 * result); //convert back to uchar4
}

//2.1 - Filter Red
kernel void filter_r(global const uchar4* A, global uchar4* B) {
	int id = get_global_id(0);
	B[id] = A[id];
	B[id].y = 0;
	B[id].z = 0;

}
//2.2 - Invert
kernel void invert(global const uchar4* A, global uchar4* B) {
	int id = get_global_id(0);
	B[id] = A[id];
	B[id].x = 255 - B[id].x;
	B[id].y = 255 - B[id].y;
	B[id].z = 255 - B[id].z;
}

//2.4 - Convert from RGB to Grey
kernel void rgb2grey(global const uchar4* A, global uchar4* B) {
	//Y = 0.2126R + 0.7152G + 0.0722B
	int id = get_global_id(0);
	B[id] = A[id];
	B[id].x = (0.2163 * B[id].x) + (0.7152 * B[id].y) + (0.0722 * B[id].z);
	B[id].y = (0.2163 * B[id].x) + (0.7152 * B[id].y) + (0.0722 * B[id].z);
	B[id].z = (0.2163 * B[id].x) + (0.7152 * B[id].y) + (0.0722 * B[id].z);

}

//3.2.2.1 - using different Convolution filters
kernel void sharpen2D(global const uchar4* A, global uchar4* B, constant float* mask) {
	int x = get_global_id(0);
	int y = get_global_id(1);
	int width = get_global_size(0); //width in pixels
	int id = x + y * width;

	float4 result = (float4)(0.0f, 0.0f, 0.0f, 0.0f);//zero all 4 components

	for (int i = (x - 1); i <= (x + 1); i++)
		for (int j = (y - 1); j <= (y + 1); j++)
			result += convert_float4(A[i + j * width])*(float4)(mask[i - (x - 1) + (j - (y - 1)) * 3]);//convert pixel and mask values to float4

	B[id] = convert_uchar4(3 * result); //convert back to uchar4
}