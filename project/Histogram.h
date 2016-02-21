#ifndef SRC_HISTOGRAMS_HISTOGRAM_H_
#define SRC_HISTOGRAMS_HISTOGRAM_H_

#include <iostream>

/** \brief Base Histogram class
 *
 *  Base histogram implementation defining the histogram itself
 *
 *  \author mack
 */

class Histogram {
protected:
	double* histogram_alpha;
	double* histogram_phi;
	double* histogram_theta;
	double min_value;
	double max_value;
	double binsize;
	int histogramSize;

public:
	Histogram(double, double, double);
	virtual ~Histogram();

	double* getHistogramAlpha() const {
		return histogram_alpha;
	}

	double* getHistogramPhi() const {
		return histogram_phi;
	}

	double* getHistogramTheta() const {
		return histogram_theta;
	}

	int getHistogramSize() const {
		return histogramSize;
	}

	virtual void serialize(std::ostream& os) const {
		int count = 1;
		for (int i = 0; i < histogramSize; i++) {
			if (histogram_alpha[i] != 0)
				os << count << ":" << histogram_alpha[i] << " ";
			count++;
		}

		for (int i = 0; i < histogramSize; i++) {
			if (histogram_phi[i] != 0)
				os << count << ":" << histogram_phi[i] << " ";
			count++;
		}

		for (int i = 0; i < histogramSize; i++) {
			if (histogram_theta[i] != 0)
				os << count << ":" << histogram_theta[i] << " ";
			count++;
		}
	}

	friend std::ostream& operator<<(std::ostream& out, const Histogram& h) {
		out.width(4);
		out.precision(3);

		h.serialize(out);

		return out;
	}
};

#endif /* SRC_HISTOGRAMS_HISTOGRAM_H_ */
