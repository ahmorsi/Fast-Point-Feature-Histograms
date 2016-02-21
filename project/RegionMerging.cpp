#include "RegionMerging.h"
#include <set>
#include <queue>
RegionMerging::RegionMerging(double rN) {
	this->rN = rN;
}

RegionMerging::~RegionMerging() {
}

void RegionMerging::performRegionMerging(PointCloud& cloud, KdTreeNode& root,
		std::vector<PointCloud*>& clusters, std::vector<Point*>& centroids) {

	//TODO: Implement region merging algorithm
	//store in clusters the new point regions and in centroids the centroids of those regions

	std::queue<Point*> open_list;
	std::set<Point*>::iterator it;
	std::set<Point*> visited;
	//PointCloud* cluster;
	for(int i=0;i<cloud.getSize();++i)
	{
		Point* curPnt = cloud.getPoints()[i];
		it = visited.find(curPnt);
		if(it != visited.end())
			continue;
		PointCloud* cluster = new PointCloud();
		open_list.push(curPnt);
		visited.insert(curPnt);
		cluster->addPoint(new Point(*curPnt));
		while(!open_list.empty())
		{
			Point* queryPnt = open_list.front();
			open_list.pop();
			std::vector<Point*> neigbhors;
			root.fixedRadiusSearch(queryPnt,rN,neigbhors);
			for(int k=0;k<neigbhors.size();++k)
			{
				Point* curNeigbhor = neigbhors[k];
				it = visited.find(curNeigbhor);
				if(it == visited.end() && queryPnt->getLabel() == curNeigbhor->getLabel())
				{
					open_list.push(curNeigbhor);
					cluster->addPoint(new Point(*curNeigbhor));
					visited.insert(curNeigbhor);
				}
			}
		}
		Point*centroid = cluster->getCentroid();
		centroids.push_back(centroid);
		clusters.push_back(cluster);
	}
}
