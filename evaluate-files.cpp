#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include "boost/filesystem.hpp"
#include <fstream>
#include <stdexcept>

/**
 * Evaluate input data after post processing
 *
 * \author: mack
 */

int main(int argc, char **argv) {

	std::string processedDir = "data/post_processed";

	double rN = 5;

	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-help") == 0) {
			std::cout << "The following parameters are available: \n"
					<< "-iPP: The path to the post processed file directory for test data (default: data/post_processed) \n"
					<< std::endl;
			return 1;
		}

		if (strcmp(argv[i], "-rN") == 0) {
			rN = atof(argv[++i]);
		} else if (strcmp(argv[i], "-iPP") == 0) {
			processedDir = argv[++i];
		}
	}

	if (!boost::filesystem::exists(processedDir))
		std::cout << " The filepath " << processedDir
				<< " does not exist. You can specify another directory using input parameter -iPP."
				<< std::endl;

	boost::filesystem::directory_iterator end_itr_pp;
	double tp = 0;
	double fp = 0;
	double tn = 0;
	double fn = 0;
	for (boost::filesystem::directory_iterator itr(processedDir);
			itr != end_itr_pp; ++itr) {
		if (itr->path().leaf().string().find(".xyz") != std::string::npos) {
			int correctLabel = 0;
			if (itr->path().filename().string().find("berries")
					!= std::string::npos) {
				correctLabel = 1;
			} else if (itr->path().filename().string().find("stem")
					!= std::string::npos) {
				correctLabel = -1;
			}
			std::ifstream infile(itr->path().c_str());

			if (!infile.good()) {
				throw std::invalid_argument(
						"There's something wrong with the source file at "
								+ itr->path().string());
			}

			int currentLabel;
			while (infile >> currentLabel) {
				if (correctLabel == 1) {
					if (currentLabel == correctLabel) {
						tp++;
					} else {
						fn++;
					}
				} else if (correctLabel == -1) {
					if (currentLabel == correctLabel) {
						tn++;
					} else {
						fp++;
					}
				}
			}

		}
	}

	double precision = tp / (tp + fp);
	double recall = tp / (tp + fn);

	std::cout << "precision: " << precision << std::endl;
	std::cout << "recall: " << recall << std::endl;

	std::cout << "done" << std::endl;

	return 1;
}
