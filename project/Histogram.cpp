#include "Histogram.h"
#include <stdlib.h>
#include <iostream>
#include <cmath>

Histogram::Histogram(double min_value, double max_value, double binsize) :
		min_value(min_value), max_value(max_value), binsize(binsize) {
	histogramSize = ceil((max_value - min_value) / binsize);
	histogram_alpha = (double*) calloc(histogramSize, sizeof(double));
	histogram_phi = (double*) calloc(histogramSize, sizeof(double));
	histogram_theta = (double*) calloc(histogramSize, sizeof(double));
}

Histogram::~Histogram() {
	free(histogram_alpha);
	free(histogram_phi);
	free(histogram_theta);
}
