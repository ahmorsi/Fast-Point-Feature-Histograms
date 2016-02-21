#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include "boost/filesystem.hpp"
#include "project/KdTree.h"
#include "project/PointCloud.h"
#include "project/Importer.h"
#include "project/Exporter.h"
#include "project/RegionMerging.h"

/**
 * Implementation of post processing after classifying point clouds into berries and stems.
 * Region merging is applied to make sure all parts of the point clouds are labeled the same.
 *
 * \author: mack
 */

int main(int argc, char **argv) {

	std::string sourceDir = "data/test";
	std::string prediction = "data/prediction/prediction.dat";
	std::string destinationFinal = "data/post_processed";

	double rN = 5;

	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-help") == 0) {
			std::cout << "The following parameters are available: \n"
					<< "-rN: The radius rN for the nearest neighbor search during normal computation \n"
					<< "-iTe: The path to the source directory for test data (default: data/test) \n"
					<< "-iPr: The path to the prediction file for test data (default: data/prediction/prediction.dat) \n"
					<< "-oF: The path to the target file for post processed test data (default: data/post_processed/label.dat) \n"
					<< std::endl;
			return 1;
		}

		if (strcmp(argv[i], "-rN") == 0) {
			rN = atof(argv[++i]);
		} else if (strcmp(argv[i], "-iTe") == 0) {
			sourceDir = argv[++i];
		} else if (strcmp(argv[i], "-iPr") == 0) {
			prediction = argv[++i];
		} else if (strcmp(argv[i], "-oF") == 0) {
			destinationFinal = argv[++i];
		}
	}

	if (!boost::filesystem::exists(sourceDir))
		std::cout << " The filepath " << sourceDir
				<< " does not exist. You can specify another directory using input parameter -iTe."
				<< std::endl;

	boost::filesystem::directory_iterator end_itr_test;
	int count = 0;
	for (boost::filesystem::directory_iterator itr(sourceDir);
			itr != end_itr_test; ++itr) {
		if (itr->path().leaf().string().find(".xyz") != std::string::npos) {
			Exporter exp(
					destinationFinal + std::string("/")
							+ itr->path().filename().string());
			exp.deleteOldFile();

			Importer import(itr->path().string());
			PointCloud cloud;

			std::cout << "Importing data from file " << itr->path().string()
					<< std::endl;

			//read point cloud from file
			count = import.readFileWithPrediction(prediction, count, cloud);

			//build kd-tree from cloud
			KdTreeNode root;
			root.buildTree(cloud.getPoints(), 0);

			//compute normals for all points with radius rN
			int count = 0;
			std::cout << "computing normals..." << std::endl;
			for (unsigned int i = 0; i < cloud.getSize(); i++) {
				count++;
				if (count % 1000 == 0) {
					std::cout << count << "..." << std::endl;
				}
				std::vector<Point*> neighbors;
				root.fixedRadiusSearch(cloud.getPoints()[i], rN, neighbors);
				cloud.getPoints()[i]->computeNormal(neighbors);
			}

			std::cout << "apply region merging..." << std::endl;

			std::vector<PointCloud*> clusters;
			std::vector<Point*> centroids;

			RegionMerging merging(rN);
			merging.performRegionMerging(cloud, root, clusters, centroids);

			std::cout << "size clusters: " << clusters.size() << std::endl;

			double meanRegionSize = 0;
			for (int j = 0; j < clusters.size(); j++) {
				meanRegionSize += clusters[j]->getSize();
			}
			meanRegionSize /= clusters.size();

			std::cout << "mean region size: " << meanRegionSize << std::endl;

			for (int j = 0; j < clusters.size(); j++) {
				if (clusters[j]->getSize() < meanRegionSize) {
					double minDist = 0;
					int minIndex = 0;
					for (int c = 0; c < centroids.size(); c++) {
						if (c == j)
							continue;
						if (centroids[c]->euclideanDistance(centroids[j])
								< minDist) {
							minDist = centroids[c]->euclideanDistance(
									centroids[j]);
							minIndex = c;
						}
					}
					for (int k = 0; k < clusters[j]->getSize(); k++) {
						clusters[j]->getPoints()[k]->setLabel(
								clusters[minIndex]->getPoints()[0]->getLabel());
					}
				}
			}

			exp.writeFile(clusters);

			for (int i = 0; i < clusters.size(); i++) {
				delete (clusters[i]);
				delete (centroids[i]);
			}
		}
	}

	std::cout << "done" << std::endl;

	return 1;
}
