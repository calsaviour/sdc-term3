/*
 *  utils.h
 *  
 *
 *  Created by Jeff Johnson on 9/19/10.
 *  Copyright 2010 Living Room Productions. All rights reserved.
 *
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <math.h>

#include "Kernel.h"

double INVERSE_NORMALIZER = sqrt(2*M_PI);

struct Point
{
	double row, col, intensity;
};
bool PointSorter ( Point x, Point y ) {
	return x.intensity > y.intensity;
}

struct Coordinate
{
	int row, col;
};

#define BIN_SIZE 128
#define COLOR_NUM 256
#define BIN_NUM COLOR_NUM / BIN_SIZE
typedef unsigned int s_label;
typedef double RGB_HIST[BIN_NUM][BIN_NUM][BIN_NUM];
typedef double INNER_CPT[4][2];
typedef double BORDER_CPT[2][2];

double SKY_PRIOR = 0.5;

/**
INNER_CPT inner_table = {
	{0.95, 0.05},
	{0.8, 0.2},
	{0.2, 0.8},
	{0.05, 0.95}
};
/**/

INNER_CPT inner_table = {
	{0.9, 0.1},
	{0.4, 0.6},
	{0.4, 0.6},
	{0.1, 0.9}
};

BORDER_CPT top_row_table = {
	{0.8, 0.2},
	{0.25, 0.75}
};

BORDER_CPT left_column_table = {
	{0.9, 0.1},
	{0.1, 0.9}
};

void init_plane( SDoublePlane& plane ) {
	for ( size_t i=0; i<plane.rows(); i++ ) {
		for ( size_t j=0; j<plane.cols(); j++ ) {
			plane[i][j] = 0.;
		}
	}
}

void print_plane( SDoublePlane& plane ) {
	for ( size_t i=0; i<plane.rows(); i++ ) {
		for ( size_t j=0; j<plane.cols(); j++ ) {
			std::cout << "plane[" << i << "][" << j << "]: " << plane[i][j] << std::endl;
		}
	}
}

void accumulate_planes( SDoublePlane& accumulated_red,
						 SDoublePlane& accumulated_green,
						 SDoublePlane& accumulated_blue,
						 SDoublePlane& input_red,
						 SDoublePlane& input_green,
						 SDoublePlane& input_blue ) {
	
	for ( size_t i=0; i<accumulated_red.rows(); i++ ) {
		for ( size_t j=0; j<accumulated_red.cols(); j++ ) {
			accumulated_red[i][j] = accumulated_red[i][j] + input_red[i][j];
			accumulated_green[i][j] = accumulated_green[i][j] + input_green[i][j];
			accumulated_blue[i][j] = accumulated_blue[i][j] + input_blue[i][j];
		}
	}
	
}

double rand_double() {
	// assume it's already been seeded
	return (double)rand() / (double)RAND_MAX;
}

void initializeHistogram( RGB_HIST& h, double val ) {
	for ( size_t i=0; i<BIN_NUM; i++ ) {
		for ( size_t j=0; j<BIN_NUM; j++ ) {
			for ( size_t k=0; k<BIN_NUM; k++ ) {
				h[i][j][k] = val;
			}
		}
	}
}
void initializeHistogram( RGB_HIST& h ) {
	initializeHistogram( h, 0. );
}

void printHistogram( RGB_HIST& h ) {
	double sum = 0.;
	for ( size_t i=0; i<BIN_NUM; i++ ) {
		for ( size_t j=0; j<BIN_NUM; j++ ) {
			for ( size_t k=0; k<BIN_NUM; k++ ) {
				std::cout << "h[" << i << "][" << j << "][" << k << "]: " << h[i][j][k] << std::endl;
				sum += h[i][j][k];
			}
		}
	}
	std::cout << "sum: " << sum << std::endl;
}

void rgb2h_indices( size_t& r_index, size_t& g_index, size_t& b_index, double r, double g, double b ) {

	r_index = (size_t)(r / BIN_SIZE);
	g_index = (size_t)(g / BIN_SIZE);
	b_index = (size_t)(b / BIN_SIZE);
	
}

void h_indices2rgb( double& r, double& g, double& b, size_t r_index, size_t g_index, size_t b_index ) {
	
	size_t half_bin_size = BIN_SIZE / 2;
	
	r = (double)( r_index * BIN_SIZE + half_bin_size );
	g = (double)( g_index * BIN_SIZE + half_bin_size );
	b = (double)( b_index * BIN_SIZE + half_bin_size );
	
}

double get_evidence( size_t i,
				  size_t j,
				  const SDoublePlane& evidence_red_plane,
				  const SDoublePlane& evidence_green_plane,
				  const SDoublePlane& evidence_blue_plane,
				  const RGB_HIST& h ) {
	size_t r_index;
	size_t g_index;
	size_t b_index;
	rgb2h_indices( r_index, g_index, b_index, evidence_red_plane[i][j], evidence_green_plane[i][j], evidence_blue_plane[i][j] );

	return h[r_index][g_index][b_index];
}

void sample_histogram( size_t& i, size_t& j, size_t& k, double& p, const RGB_HIST& h ) {
	
	// random number (assume it's already been seeded)
	double r = rand_double();
	
	// find the interval that this r falls into
	p = 0.;
	for ( i=0; i<BIN_NUM; i++ ) {
		for ( j=0; j<BIN_NUM; j++ ) {
			for ( k=0; k<BIN_NUM; k++ ) {
				
				p += h[i][j][k];
				if ( p >= r ) {
					return;
				}
				
			}
		}
	}
	
}

/**
 * Scale pixel values for an image.
 *
 * @param	input			The base image
 * @param	scale			The value scale factor
 * @return					The scaled value image
 */
SDoublePlane ScaleImageValues( const SDoublePlane &input ) {
	
	double scale = 255;
	
	// Create an image to store the crop
	SDoublePlane scaled_image( input.rows(), input.cols() );
	
	// Do the normalization
	for ( unsigned int i=0; i<scaled_image.rows(); i++ ) {
		for ( unsigned int j=0; j<scaled_image.cols(); j++ ) {
			scaled_image[i][j] = (int)(input[i][j] * scale);
		}
	}
	
	return scaled_image;
}

/**
 * Normalize an image that has been read in based on the values at the pixels
 *
 * @param	input			The base image
 * @return					The normalized image
 */
SDoublePlane NormalizeImage( const SDoublePlane &input ) {
	
	// Create an image to store the crop
	SDoublePlane normed_image( input.rows(), input.cols() );
	
	double max = 0.;
	for ( size_t i=0; i<input.rows(); i++ ) {
		for ( size_t j=0; j<input.cols(); j++ ) {
			double val = input[i][j];
			if ( val > max ) {
				max = val;
			}
		}
	}
	
	if ( max == 0. ) {
		return normed_image;
	}
	
	for ( size_t i=0; i<input.rows(); i++ ) {
		for ( size_t j=0; j<input.cols(); j++ ) {
			
			double val = input[i][j];
			normed_image[i][j] = val / max;
			
		}
	}
	
	return normed_image;
}


/**
 * Mirror pad an image to a given number of pixels
 *
 * @param	input			The base image
 * @param	padding			The number of pixels to pad
 * @return					The padded image
 */
SDoublePlane PadImage( const SDoublePlane &input, unsigned int pad ) {
	
	// The pad cannot exceed the dimensions of the image
	if ( pad > input.rows() ) {
		pad = input.rows();
	}
	if ( pad > input.cols() ) {
		pad = input.cols();
	}
	
	// Create a new image of size input + pad
	SDoublePlane padded( input.rows()+(2*pad), input.cols()+(2*pad) );
	
	// Pad top, bottom, and sides
	for ( unsigned int i=0; i<pad; i++ ) {
		
		for ( unsigned int j=0; j<input.cols(); j++ ) {
			// top
			padded[pad-(i+1)][j+pad] = input[i][j];
			
			// bottom
			padded[(input.rows()+pad)+i][j+pad] = input[(input.rows()-1)-i][j];
		}

		for ( unsigned k=0; k<input.rows(); k++ ) {
			// left
			padded[k+pad][pad-(i+1)] = input[k][i];
			
			// right
			padded[k+pad][(input.cols()+pad)+i] = input[k][(input.cols()-1)-i];
		}
	}
	
	// Pad diagonal
	for ( unsigned int i=0; i<pad; i++ ) {
		for ( unsigned int j=0; j<pad; j++ ) {
			
			// Top left
			padded[i][j] = input[pad-1-j][pad-1-i];
			
			// Top right
			padded[i][padded.cols()-1-j] = input[pad-1-j][input.cols()-1-pad+i];
			
			// Bottom left
			padded[padded.rows()-1-i][pad-1-j] = input[input.rows()-1-j][pad-1-i];
			
			// Bottom right
			padded[padded.rows()-1-i][padded.cols()-pad+j] = input[input.rows()-1-j][input.cols()-1-pad+i];
		}
	}
	
	// Copy the rest
	for ( unsigned int i=0; i<input.rows(); i++ ) {
		for ( unsigned int j=0; j<input.cols(); j++ ) {
			padded[i+pad][j+pad] = input[i][j];
		}
	}
	
	return padded;
}

/**
 * Quick and dirty cropping method
 *
 * @param	input			The base image
 * @param	pad				The amount to trim from edges
 * @return					The cropped image
 */
SDoublePlane CropImage( const SDoublePlane &input, unsigned int pad ) {
	
	// Create an image to store the crop
	SDoublePlane cropped_image( input.rows()-(2*pad), input.cols()-(2*pad) );
	
	// Do the crop
	for ( unsigned int i=0; i<cropped_image.rows(); i++ ) {
		for ( unsigned int j=0; j<cropped_image.cols(); j++ ) {
			cropped_image[i][j] = input[i+pad][j+pad];
		}
	}
	
	return cropped_image;
}

#endif
