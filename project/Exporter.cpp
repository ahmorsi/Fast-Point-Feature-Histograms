#include "Exporter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

Exporter::Exporter(std::string fn) :
		filename(fn) {
}

Exporter::~Exporter() {
}

std::string Exporter::getFilename() {
	return filename;
}

void Exporter::deleteOldFile() {
	remove(filename.c_str());
}

void Exporter::writeFile(PointCloud& cloud) {
	std::ofstream outfile;
	outfile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

	if (!outfile.good()) {
		throw std::invalid_argument(
				"There's something wrong with the source file at " + filename);
	}

	for (unsigned int i = 0; i < cloud.getSize(); i++) {
		if (cloud.getPoints()[i]->getFastPointFeatureHistogram()->getNumberOfEntries()
				!= 0)
			outfile << cloud.getPoints()[i]->getLabel() << " "
					<< *(cloud.getPoints()[i]->getFastPointFeatureHistogram())
					<< std::endl;
	}

	outfile.close();
}

void Exporter::writeFile(std::vector<PointCloud*>& clusters) {
	std::ofstream outfile;
	outfile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

	if (!outfile.good()) {
		throw std::invalid_argument(
				"There's something wrong with the source file at " + filename);
	}

	for (unsigned int i = 0; i < clusters.size(); i++) {
		for (unsigned int j = 0; j < clusters[i]->getSize(); j++) {
			outfile << clusters[i]->getPoints()[j]->getLabel() << std::endl;
		}
	}

	outfile.close();
}
