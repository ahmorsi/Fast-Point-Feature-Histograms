#ifndef REGIONMERGING_H_
#define REGIONMERGING_H_

#include "PointCloud.h"
#include "KdTree.h"
#include <vector>

/** \brief Implementation of the region merging algorithm
 *
 *  The region merging approach is applied to a set of labeled points.
 *
 *  \author mack
 */


class RegionMerging {
	double rN;
public:
	RegionMerging(double);
	virtual ~RegionMerging();

	//perform region merging on the points in the PointCloud using the k-d tree. Return clusters and centroids of the clusters
	void performRegionMerging(PointCloud&, KdTreeNode&,
			std::vector<PointCloud*>&, std::vector<Point*>&);
};

#endif /* REGIONMERGING_H_ */
