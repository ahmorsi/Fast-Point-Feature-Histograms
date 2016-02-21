#ifndef SRC_HISTOGRAMS_FPFH_H_
#define SRC_HISTOGRAMS_FPFH_H_

#include "Histogram.h"
#include "SPFH.h"

class SPFH;

/** \brief Implementation of a FPFH
 *
 *  A Fast Point Feature Histogram computes its values from the point's neighbor's SPFHs, the
 *  number of neighbors and a weight for each neighbor.
 *
 *  \author mack
 */

class FPFH: public Histogram {
	int numberOfEntries;

public:
	FPFH(double, double, double);
	virtual ~FPFH();

	int getNumberOfEntries() const {
		return numberOfEntries;
	}

	//add entry to FPFH, consisting of a SPFH, the number of neighbors k and the corresponding weight w_k
	void addToHistogram(SPFH*, int, double);

	friend std::ostream& operator<<(std::ostream& out, const FPFH& h) {
		h.serialize(out);
		return out;
	}
};

#endif /* SRC_HISTOGRAMS_FPFH_H_ */
