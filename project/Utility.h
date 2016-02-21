#ifndef SRC_UTILITY_UTILITY_H_
#define SRC_UTILITY_UTILITY_H_

#include "Point.h"
#include <Eigen/Eigenvalues>

/** \brief Collection of helper methods
 *
 *  Contains some basic methods for computing a covariance matrix
 *
 *  \author mack
 */

//compute the center of gravity in the set of points
Point* computeCenterOfGravity(std::vector<Point*> points) {
	double coord[3] = {0};

	for(int i=0;i<points.size();++i)
	{
		coord[0] += points[i]->getX();
		coord[1] += points[i]->getY();
		coord[2] += points[i]->getZ();
	}
	coord[0] /= points.size();
	coord[1] /= points.size();
	coord[2] /= points.size();
	return new Point(0, coord[0], coord[1], coord[2]);
}

//compute the covariance matrix for the set of points and a given center of gravity
void computeCovarianceMatrix(std::vector<Point*> neighbors,
		Point* centerOfGravity, Eigen::Matrix3f& covariance) {

	for(int i=0;i<neighbors.size();++i)
	{
		covariance += ((neighbors[i]->getCoordinates()*neighbors[i]->getCoordinates().transpose())-
				(centerOfGravity->getCoordinates()*centerOfGravity->getCoordinates().transpose()));
	}

	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			covariance(i,j)/=neighbors.size();
}

#endif /* SRC_UTILITY_UTILITY_H_ */
