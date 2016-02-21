#include <gtest/gtest.h>
#include <boost/random.hpp>
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"

TEST(SVMLightTest, Training) {
	system("cd ../");

	if (!boost::filesystem::exists("data/model/model.dat")
			|| !boost::filesystem::exists("data/prediction/prediction.dat")) {
		system("./create-histograms");
	}
	system("./classify-histograms");

	std::ifstream infile("data/model/model.dat");

	ASSERT_TRUE(infile.good())<< "Something is wrong with model file at data/model/model.dat";

	infile.close();

	std::ifstream infile2("data/prediction/prediction.dat");

	ASSERT_TRUE(infile2.good())<< "Something is wrong with prediction file at data/prediction/prediction.dat";

	infile2.close();

}

