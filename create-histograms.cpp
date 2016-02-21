#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "boost/filesystem.hpp"
#include <cstring>
#include "project/Importer.h"
#include "project/Exporter.h"
#include "project/PointCloud.h"
#include <cmath>

#include "project/KdTree.h"
#include "project/SPFH.h"

/**
 * Implementation of histogram computation following these steps:
 * 1.) read point cloud from input file
 * 2.) build kd-tree from point cloud
 * 3.) compute normals for all points in point cloud
 * 4.) compute SPFH for every point in point cloud
 * 5.) compute FPFH for every point in point cloud
 * 6.) export histogram representation to feature file
 *
 * \author: mack
 */
void computeHistogramsForPath(std::string path, double rN, double rH,
		double minsize, double maxsize, double binsize, Exporter exp) {
	Importer import(path);
	PointCloud cloud;

	std::cout << "Importing data from file " << path << std::endl;

	//read point cloud from file
	import.readFile(cloud);

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

	//compute SPFHs for all points with radius rH
	count = 0;
	std::cout << "computing SPFHs..." << std::endl;
	for (unsigned int i = 0; i < cloud.getSize(); i++) {
		count++;
		if (count % 1000 == 0) {
			std::cout << count << "..." << std::endl;
		}
		std::vector<Point*> neighbors;
		root.fixedRadiusSearch(cloud.getPoints()[i], rH, neighbors);

		SPFH* s = new SPFH(minsize, maxsize, binsize);
		for (unsigned int j = 0; j < neighbors.size(); j++) {
			if (cloud.getPoints()[i] == neighbors[j]) {
				continue;
			}
			Features* f = new Features(cloud.getPoints()[i], neighbors[j]);
			s->addToHistogram(f);
			delete f;
		}
		cloud.getPoints()[i]->setSimplePointFeatureHistogram(s);
	}

	//compute SPFHs for all points with radius rH
	count = 0;
	std::cout << "computing FPFHs..." << std::endl;
	for (unsigned int i = 0; i < cloud.getSize(); i++) {
		count++;
		if (count % 1000 == 0) {
			std::cout << count << "..." << std::endl;
		}
		std::vector<Point*> neighbors;
		root.fixedRadiusSearch(cloud.getPoints()[i], rH, neighbors);
		FPFH* f = new FPFH(minsize, maxsize, binsize);
		f->addToHistogram(
				cloud.getPoints()[i]->getSimplePointFeatureHistogram(), 1, 1);

		for (unsigned int j = 0; j < neighbors.size(); j++) {
			if (cloud.getPoints()[i] == neighbors[j]) {
				continue;
			}
			f->addToHistogram(neighbors[j]->getSimplePointFeatureHistogram(),
					neighbors.size(),
					cloud.getPoints()[i]->euclideanDistance(neighbors[j]));
		}
		cloud.getPoints()[i]->setFastPointFeatureHistogram(f);

	}

	//write histograms to file
	std::cout << "Writing histograms in file..." << std::endl;
	exp.writeFile(cloud);
}

int main(int argc, char **argv) {
	std::string sourceDirTrain = "data/train";
	std::string sourceDirTest = "data/test";
	std::string destinationTrain = "data/histograms/train/histograms.dat";
	std::string destinationTest = "data/histograms/test/histograms.dat";
	double rN = 5;
	double rH = 2.5;
	double binsize = 0.1;
	double minsize = -M_PI;
	double maxsize = M_PI;

	//read in parameters, use standard parameters if nothing given
	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-help") == 0) {
			std::cout << "The following parameters are available: \n"
					<< "-rN: The radius rN for the nearest neighbor search during normal computation \n"
					<< "-rH: The radius rH for the nearest neighbor search during histogram computation \n"
					<< "-iTr: The path to the source directory for training data (default: data/train) \n"
					<< "-iTe: The path to the source directory for test data (default: data/test) \n"
					<< "-oTr: The path to the target file for training data (default: data/histograms/train/histograms.dat) \n"
					<< "-oTe: The path to the target file for test data (default: data/histograms/test/histograms.dat) \n"
					<< "-bin: The binsize of the histograms (default: 0.1) "
					<< std::endl;
			return 1;
		}

		if (strcmp(argv[i], "-rN") == 0) {
			rN = atof(argv[++i]);
		} else if (strcmp(argv[i], "-rH") == 0) {
			rH = atof(argv[++i]);
		} else if (strcmp(argv[i], "-iTr") == 0) {
			sourceDirTrain = argv[++i];
		} else if (strcmp(argv[i], "-iTe") == 0) {
			sourceDirTest = argv[++i];
		} else if (strcmp(argv[i], "-oTr") == 0) {
			destinationTrain = argv[++i];
		} else if (strcmp(argv[i], "-oTe") == 0) {
			destinationTest = argv[++i];
		} else if (strcmp(argv[i], "-bin") == 0) {
			binsize = atof(argv[++i]);
		}
	}

	//compute histograms for training files
	Exporter expTrain(destinationTrain);
	expTrain.deleteOldFile();

	if (!boost::filesystem::exists(sourceDirTrain))
		std::cout << " The filepath " << sourceDirTrain
				<< " does not exist. You can specify another directory using input parameter -i."
				<< std::endl;

	boost::filesystem::directory_iterator end_itr_train;
	for (boost::filesystem::directory_iterator itr(sourceDirTrain);
			itr != end_itr_train; ++itr) {
		if (itr->path().leaf().string().find(".xyz") != std::string::npos) {
			computeHistogramsForPath(itr->path().string(), rN, rH, minsize,
					maxsize, binsize, expTrain);
		}
	}

	//compute histograms for test files
	Exporter expTest(destinationTest);
	expTest.deleteOldFile();

	if (!boost::filesystem::exists(sourceDirTest))
		std::cout << " The filepath " << sourceDirTest
				<< " does not exist. You can specify another directory using input parameter -i."
				<< std::endl;
	boost::filesystem::directory_iterator end_itr_test; // default construction yields past-the-end
	for (boost::filesystem::directory_iterator itr(sourceDirTest);
			itr != end_itr_test; ++itr) {
		if (itr->path().leaf().string().find(".xyz") != std::string::npos) {
			computeHistogramsForPath(itr->path().string(), rN, rH, minsize,
					maxsize, binsize, expTest);
		}
	}

	std::cout << "done" << std::endl;

	return 1;
}
