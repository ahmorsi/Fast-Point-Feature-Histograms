#include "KdTree.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>

KdTreeNode::KdTreeNode() :
		location(), right_node(), left_node(), depth(0) {
}

KdTreeNode::~KdTreeNode() {
	delete (right_node);
	delete (left_node);
}

int KdTreeNode::getDepth() const {
	return depth;
}

std::vector<Point*> KdTreeNode::getSuccessors() const {
	return successors;
}

void KdTreeNode::buildTree(std::vector<Point*> points, int depth) {
	this->depth = depth;
	int size = points.size();

	if (size == 1) {
		location = points[0];
		return;
	} else if (size == 0) {
		return;
	}

	int axis = depth % 3;
	MergeSort(axis, points, size);

	int mid = (size - 1) / 2;
	location = points[mid];

	std::vector<Point*> left, right;
	left = subset(points, 1, mid);
	right = subset(points, 0, mid);

	for (int i = 0; i < left.size(); ++i)
		successors.push_back(left[i]);
	for (int i = 0; i < right.size(); ++i)
		successors.push_back(right[i]);
	this->left_node = new KdTreeNode();
	this->right_node = new KdTreeNode();
	this->left_node->buildTree(left, depth + 1);
	this->right_node->buildTree(right, depth + 1);
}

void KdTreeNode::fixedRadiusSearch(Point* queryPoint, double radius,
		std::vector<Point*>& neighbors) {
	if (location == NULL) {
		return;
	}

	if (PointInSphere(location, radius, queryPoint))
		neighbors.push_back(location);

	if (this->left_node != NULL) {
		if (this->left_node->IsAllPointsInSphere(radius, queryPoint)) {
			for (int i = 0; i < this->left_node->successors.size(); ++i)
				neighbors.push_back(this->left_node->successors[i]);
			if (this->left_node->location != NULL && PointInSphere(this->left_node->location, radius, queryPoint))
				neighbors.push_back(this->left_node->location);
		} else if (this->left_node->IntersectSphere(radius, queryPoint)) {
			this->left_node->fixedRadiusSearch(queryPoint, radius, neighbors);
		}
	}
	if (this->right_node != NULL) {
		if (this->right_node->IsAllPointsInSphere(radius, queryPoint)) {
			for (int i = 0; i < this->right_node->successors.size(); ++i)
				neighbors.push_back(this->right_node->successors[i]);
			if (this->right_node->location != NULL && PointInSphere(this->right_node->location, radius, queryPoint))
				neighbors.push_back(this->right_node->location);
		} else if (this->right_node->IntersectSphere(radius, queryPoint)) {
			this->right_node->fixedRadiusSearch(queryPoint, radius, neighbors);
		}
	}

}
bool KdTreeNode::PointInSphere(Point* p, double radius, Point* center) {
	double dist = center->euclideanDistance(p);
	return dist <= radius;
}
bool KdTreeNode::IsAllPointsInSphere(double radius, Point* center) {
	for (int i = 0; i < successors.size(); ++i) {
		if (!PointInSphere(successors[i], radius, center))
			return false;
	}
	return true;
}
bool KdTreeNode::IntersectSphere(double radius, Point* center) {
	for (int i = 0; i < successors.size(); ++i) {
		if (PointInSphere(successors[i], radius, center))
			return true;
	}
	return this->location != NULL
			&& PointInSphere(this->location, radius, center);
}
void KdTreeNode::MergeSort(int axis, std::vector<Point*>& points, int curSize) {

	//TODO: Implement Merge Sort, sort points with respect to the axis (0 = x-axis, 1 = y-axis, 2 = z-axis)
	if (curSize <= 1)
		return;

	std::vector<Point*> left = subset(points, 1, curSize / 2);
	std::vector<Point*> right = subset(points, 0, curSize / 2 - 1);
	MergeSort(axis, left, left.size());
	MergeSort(axis, right, right.size());

	int l = 0, r = 0;
	for (int i = 0; i < curSize; ++i) {
		if (l < left.size()
				&& (r == right.size()
						|| left[l]->getAxisVal(axis)
								< right[r]->getAxisVal(axis)))
			points[i] = left[l++];
		else
			points[i] = right[r++];
	}
}

std::vector<Point*> KdTreeNode::subset(std::vector<Point*> points, int before,
		int index) {

	std::vector<Point*>::const_iterator first;
	std::vector<Point*>::const_iterator last;

	if (before == 1) {
		first = points.begin();
		last = points.begin() + index;
	} else {
		first = points.begin() + index + 1;
		last = points.end();
	}

	return std::vector<Point*>(first, last);
}
