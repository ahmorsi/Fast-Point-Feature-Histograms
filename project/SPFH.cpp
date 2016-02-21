#include "SPFH.h"
#include <math.h>

SPFH::SPFH(double min_value, double max_value, double binsize) :
		Histogram(min_value, max_value, binsize) {
}

SPFH::~SPFH() {
}

void SPFH::addToHistogram(Features* f) {

	//TODO: add alpha, phi, theta values from f to histogram
	if (f->getAlpha() >= min_value && f->getAlpha() <= max_value) {
		int alphaIdx = (int)(f->getAlpha()-min_value) / binsize;
		++histogram_alpha[alphaIdx];
	}else
		std::cout<<"Alpha Binning\n";
	if (f->getPhi() >= min_value && f->getPhi() <= max_value) {
		int phiIdx = (int)(f->getPhi()-min_value) / binsize;
		++histogram_phi[phiIdx];
	}
	if (f->getTheta() >= min_value && f->getTheta() <= max_value) {
		int thetaIdx = (int)(f->getTheta()-min_value) / binsize;
		++histogram_theta[thetaIdx];
	}
}
