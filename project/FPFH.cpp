#include "FPFH.h"

FPFH::FPFH(double min_value, double max_value, double binsize) :
		Histogram(min_value, max_value, binsize), numberOfEntries(0) {
}

FPFH::~FPFH() {
}

//compute FPFH entries from a neighbors SPFH, the number of neighbors k and the neighbor's weights w_k
void FPFH::addToHistogram(SPFH* simpleHistogram, int k, double wk) {
	int count = 0;

	//TODO: compute histogram entries for the FPFH. If entry is != 0, increase count by 1

		for (int i = 0; i < histogramSize; ++i) {
			histogram_alpha[i] +=
					(simpleHistogram->getHistogramAlpha()[i] / wk);
			histogram_phi[i] += (simpleHistogram->getHistogramPhi()[i] / wk);
			histogram_theta[i] +=
					(simpleHistogram->getHistogramTheta()[i] / wk);
			if(!histogram_alpha[i]|| !histogram_phi[i]||!histogram_theta[i])
							++ count;
		}
		for (int i = 0; i < histogramSize; ++i) {
			histogram_alpha[i] /= k;
			histogram_phi[i] /= k;
			histogram_theta[i] /= k;

		}

	numberOfEntries = count;
}
