#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>
#include "Point.h"

/** \brief Implementation of a kd-tree
 *
 *  The kd-tree is build for a set of points and enables fast nearest neighbor search in a
 *  given radius
 *
 *  \author mack
 */

class KdTreeNode {
	int depth;
	std::vector<Point*> successors;

	void MergeSort(int axis, std::vector<Point*>&, int);
	std::vector<Point*> subset(std::vector<Point*>, int, int);
	static bool PointInSphere(Point* p, double radius,Point* center);
	bool IsAllPointsInSphere(double radius,Point* center);
	bool IntersectSphere(double radius,Point* center);
public:
	Point* location;
	KdTreeNode* right_node;
	KdTreeNode* left_node;

	KdTreeNode();
	virtual ~KdTreeNode();

	int getDepth() const;
	std::vector<Point*> getSuccessors() const;

	//build a kd-tree from a point set, initial call with depth = 0
	void buildTree(std::vector<Point*> points, int depth);

	//execute fixed radius search around a given point p in radius k, stores neighbors in result set
	void fixedRadiusSearch(Point* p, double k, std::vector<Point*>& neighbors);

	friend std::ostream& operator<<(std::ostream& out, const KdTreeNode& k) {
		out.width(4);
		out.precision(3);

		if (k.location == NULL)
			return out;
		out << *k.location << "\n";

		if (k.right_node != NULL) {
			out << "depth: " << k.getDepth() + 1 << " right tree: \n";
			out << *(k.right_node) << "\n";
		}

		if (k.left_node != NULL) {
			out << "depth: " << k.getDepth() + 1 << " left tree: \n";
			out << *(k.left_node) << "\n";
		}

		return out;
	}
};

#endif /* KDTREE_H_ */
