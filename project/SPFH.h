#ifndef SRC_HISTOGRAMS_SPFH_H_
#define SRC_HISTOGRAMS_SPFH_H_

#include "Histogram.h"
#include "Features.h"

class Features;

/** \brief Implementation of a SPFH
 *
 *  A Simplified Point Feature Histogram contains the alpha, phi and theta features computed
 *  between the point and its neighbors
 *
 *
 *  \author mack
 */

class SPFH : public Histogram{
public:
	SPFH(double, double, double);
	virtual ~SPFH();

	//add alpha, phi, theta feature to the SPFH
	void addToHistogram(Features*);

	friend std::ostream& operator<<(std::ostream& out, const SPFH& h){
		h.serialize(out);
		return out;
	}
};

#endif /* SRC_HISTOGRAMS_SPFH_H_ */
