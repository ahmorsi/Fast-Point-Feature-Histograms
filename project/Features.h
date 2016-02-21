#ifndef SRC_FEATURES_FEATURES_H_
#define SRC_FEATURES_FEATURES_H_

#include "Point.h"

class Point;

/** \brief Implementation of the features alpha, phi and theta used for the SPFH
 *
 *  A feature consists of alpha, phi and theta value computed for two given points. These are
 *  used to compute a SPFH.
 *
 *  \author mack
 */

class Features {
	double alpha;
	double phi;
	double theta;

	//compute alpha, phi and theta features for the two points given
	void computeFeatures(Point*, Point*);

public:
	Features(Point*, Point*);
	virtual ~Features();

	double getAlpha(){
		return alpha;
	}

	double getPhi() {
		return phi;
	}

	double getTheta() {
		return theta;
	}
};

#endif /* SRC_FEATURES_FEATURES_H_ */
