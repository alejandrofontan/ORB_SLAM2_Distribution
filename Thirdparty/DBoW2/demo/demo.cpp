/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>
#include <bitset>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace DBoW2;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

enum DescriptorType {
    ORB = 0,
    AKAZE16 = 1,
    AKAZE32 = 2,
    AKAZE61 = 3,
    BRISK = 4,
    SIFT = 5,
    KAZE = 6,
    SURF = 7,
};

#define AKAZE32_FEATURE

#ifdef ORB_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::ORB
#define DESCRIPTOR_CV cv::ORB
#define DESCRIPTOR_F DBoW2::FORB
#define DESCRIPTOR_FORMAT cv::Mat
#endif

#ifdef AKAZE16_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::AKAZE16
#define DESCRIPTOR_CV cv::AKAZE
#define DESCRIPTOR_F DBoW2::FAkaze16
#define DESCRIPTOR_FORMAT cv::Mat
#endif

#ifdef AKAZE32_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::AKAZE32
#define DESCRIPTOR_CV cv::AKAZE
#define DESCRIPTOR_F DBoW2::FAkaze32
#define DESCRIPTOR_FORMAT cv::Mat
#endif

#ifdef AKAZE61_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::AKAZE61
#define DESCRIPTOR_CV cv::AKAZE
#define DESCRIPTOR_F DBoW2::FAkaze61
#define DESCRIPTOR_FORMAT cv::Mat
#endif

#ifdef BRISK_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::BRISK
#define DESCRIPTOR_CV cv::BRISK
#define DESCRIPTOR_F DBoW2::FBrisk
#define DESCRIPTOR_FORMAT cv::Mat
#endif



#ifdef SURF_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::SURF
#define DESCRIPTOR_CV cv::xfeatures2d::SURF
#define DESCRIPTOR_F DBoW2::FSurf64
#define DESCRIPTOR_FORMAT vector<float>
#endif

#ifdef KAZE_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::KAZE
#define DESCRIPTOR_CV cv::KAZE
#define DESCRIPTOR_F DBoW2::FKAZE
#define DESCRIPTOR_FORMAT vector<float>
#endif

#ifdef SIFT_FEATURE
#define DESCRIPTOR_TYPE DescriptorType::SIFT
#define DESCRIPTOR_CV cv::SIFT
#define DESCRIPTOR_F DBoW2::FSift
#define DESCRIPTOR_FORMAT vector<int>
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void loadFeatures(vector<vector<DESCRIPTOR_FORMAT>> &features, const std::vector<std::string>& imagePaths);
void changeStructure(const cv::Mat &plain, vector<DESCRIPTOR_FORMAT> &out);
void testVocCreation(const vector<vector<DESCRIPTOR_FORMAT>> &features);

void testDatabase(const vector<vector<DESCRIPTOR_FORMAT>> &features);
std::vector<std::vector<std::string>> read_txt(const std::string &filePath, const size_t &numCols, char delimiter ,int headerRows);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
int numberOfImages{};
string savePath{""};
string vocName{""};
string descriptor{""};
DescriptorType descriptorType{DESCRIPTOR_TYPE};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

std::vector<std::vector<std::string>> read_txt(const std::string &filePath, const size_t &numCols, char delimiter ,int headerRows) {
    std::vector<std::vector<std::string>> txtFile{};
    std::ifstream file(filePath);
    std::string line, word;

    if (file.good()) {
        file.clear();
        file.seekg(0, std::ios::beg);

        // Skip the rows of the header
        for (int jRow{0}; jRow < headerRows; jRow++) {
            getline(file, line);
        }

        // Stpre rows and works in txtFile vectors
        while (getline(file, line)) {
            std::stringstream line_stream(line);
            std::vector <std::string> row{};
            for (int jRow{0}; jRow < numCols; jRow++) {
                getline(line_stream, word, delimiter);
                row.push_back(word);
            }
            txtFile.push_back(row);
        }
    } else {
        // throw std::invalid_argument("File not found : " + filePath);
        cout << "File not found : " + filePath<< endl;
    }
    return txtFile;
}

int main(int argc,char **argv)
{
    descriptor=argv[1];
    savePath = argv[2];
    string imageFolder = argv[3];
    vocName = descriptor + "_DBoW2";

    bool correctCompilation{false};
    switch (descriptorType) {
        case ORB:{
            correctCompilation = (descriptor == "orb");
            break;
        }
        case AKAZE16:{
            correctCompilation = (descriptor == "akaze16");
            break;
        }
        case AKAZE32:{
            correctCompilation = (descriptor == "akaze32");
            break;
        }
        case AKAZE61:{
            correctCompilation = (descriptor == "akaze61");
            break;
        }
        case BRISK:{
            correctCompilation = (descriptor == "brisk");
            break;
        }
        case KAZE:{
            correctCompilation = (descriptor == "kaze");
            break;
        }
        case SURF:{
            correctCompilation = (descriptor == "surf");
            break;
        }
        case SIFT:{
            correctCompilation = (descriptor == "sift");
            break;
        }
    }

    if(!correctCompilation)
    {
        cout << "Demo compiled with descriptor type : " << descriptorType << endl;
        cout << "Recompile with : " << descriptor << endl;
        terminate();
    }
    std::vector<std::vector<std::string>> imagesTxt = read_txt(imageFolder,1,' ',0);
    std::vector<std::string> imagePaths;
    for(size_t imageId{0}; imageId < imagesTxt.size(); imageId = imageId + 6)
        imagePaths.push_back(imagesTxt[imageId][0]);

    numberOfImages = imagePaths.size();
    cout << "[demo] Number Of Images = " << numberOfImages<< endl;

    cout << "[demo] Loading features .."<< endl;
    vector<vector<DESCRIPTOR_FORMAT>> features;
    loadFeatures(features,imagePaths);

    cout << "[demo] Test Vocabulary Creation .."<< endl;
    testVocCreation(features);

    //wait();
    //testDatabase(features);

    return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<DESCRIPTOR_FORMAT>> &features, const std::vector<std::string>& imagePaths)
{
  features.clear();
  features.reserve(imagePaths.size());

#if defined(ORB_FEATURE) || defined(AKAZE61_FEATURE) || defined(BRISK_FEATURE) || defined(SURF_FEATURE) || defined(SIFT_FEATURE)
    cv::Ptr<DESCRIPTOR_CV> featureDetector = DESCRIPTOR_CV::create();
#endif

#ifdef AKAZE16_FEATURE
    cv::Ptr<DESCRIPTOR_CV> featureDetector = DESCRIPTOR_CV::create(cv::AKAZE::AKAZE::DESCRIPTOR_MLDB,16*8);
#endif

#ifdef AKAZE32_FEATURE
    cv::Ptr<DESCRIPTOR_CV> featureDetector = DESCRIPTOR_CV::create(cv::AKAZE::AKAZE::DESCRIPTOR_MLDB,32*8);
#endif

  cout << "Extracting " << descriptor <<" features..." << endl;
  int descriptorSize_ = 0;
  int descriptorType_ = 0;
  for(int i = 0; i < numberOfImages; ++i)
  {
    cv::Mat image = cv::imread(imagePaths[i], 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

#if defined(ORB_FEATURE) || defined(AKAZE16_FEATURE) || defined(AKAZE32_FEATURE) || defined(AKAZE61_FEATURE) || defined(BRISK_FEATURE) || defined(SURF_FEATURE) || defined(SIFT_FEATURE)
      featureDetector->detectAndCompute(image, mask, keypoints, descriptors);
#endif

    features.push_back(vector<DESCRIPTOR_FORMAT >());
    changeStructure(descriptors, features.back());

    descriptorSize_ = (int)descriptors.cols;
    descriptorType_ = (int)descriptors.type();

  }

  cout << "Descriptor Size in bytes: " << descriptorSize_<< endl;
  cout << "Descriptor Type: " << descriptorType_<< endl;
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<DESCRIPTOR_FORMAT> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------
void testVocCreation(const vector<vector<DESCRIPTOR_FORMAT>> &features)
{
  // branching factor and depth levels
  const int k = 10; //9
  const int L = 6; //3
  const WeightingType weight = TF_IDF;
  const ScoringType scoring = L1_NORM;

  DBoW2::TemplatedVocabulary<DESCRIPTOR_F::TDescriptor, DESCRIPTOR_F> vocabulary(k, L, weight, scoring);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  vocabulary.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << vocabulary << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  /*BowVector v1, v2;
  for(int i = 0; i < numberOfImages; i++)
  {
      vocabulary.transform(features[i], v1);
      for(int j = 0; j < numberOfImages; j++)
      {
          vocabulary.transform(features[j], v2);
          double score = vocabulary.score(v1, v2);
          cout << "Image " << i << " vs Image " << j << ": " << score << endl;
      }
  }*/

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  vocabulary.save(savePath +"/" + descriptor + "_DBoW2_voc.yml.gz");
  cout << "Done" << endl;
}

void testDatabase(const vector<vector<DESCRIPTOR_FORMAT > > &features)
{
  cout << "Creating a small database..." << endl;

  // load the vocabulary from disk
  DBoW2::TemplatedVocabulary<DESCRIPTOR_F::TDescriptor, DESCRIPTOR_F> vocabulary(savePath +"/" + descriptor + "_DBoW2_voc.yml.gz");
  DBoW2::TemplatedDatabase<DESCRIPTOR_F::TDescriptor, DESCRIPTOR_F> database(vocabulary, false, 0);

  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < numberOfImages; i++)
  {
      database.add(features[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << database << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  QueryResults ret;
  for(int i = 0; i < numberOfImages; i++)
  {
      database.query(features[i], ret, 4);

    // ret[0] is always the same image in this case, because we added it to the 
    // database. ret[1] is the second best match.

    cout << "Searching for Image " << i << ". " << ret << endl;
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
    database.save(savePath +"/" + descriptor + "_DBoW2_db.yml.gz");
  cout << "... done!" << endl;
  
  // once saved, we can load it again  
  cout << "Retrieving database once again..." << endl;
  DBoW2::TemplatedDatabase<DESCRIPTOR_F::TDescriptor, DESCRIPTOR_F> database2(savePath +"/" + descriptor + "_DBoW2_db.yml.gz");
  cout << "... done! This is: " << endl << database2 << endl;
}

// ----------------------------------------------------------------------------


