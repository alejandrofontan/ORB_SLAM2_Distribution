
#
#include <vector>

#include "FeatureExtractor.h"
#include "OrbFunctions.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

DescriptorType FeatureExtractor::descriptorType{DESCRIPTOR_TYPE};

static bool cornerComparison(pair<int,ExtractorNode*> a, pair<int,ExtractorNode*> b){
    return (a.first != b.first) ? (a.first < b.first) : (a.second->UL.x < b.second->UL.x);
}

FeatureExtractor::FeatureExtractor(int _nfeatures, float _scaleFactor, int _nlevels,
                                   int _iniThFAST, int _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
    // Allocate variables
    mvScaleFactor.resize(nlevels);
    mvInvScaleFactor.resize(nlevels);
    mvImagePyramid.resize(nlevels);

    mvScaleFactor[0] = 1.0f;
    //mvLevelSigma2[0] = 1.0f;
    for(int level = 1; level < nlevels; level++)
    {
        mvScaleFactor[level] = mvScaleFactor[level-1] * scaleFactor;
        //mvLevelSigma2[level] = mvScaleFactor[level] * mvScaleFactor[level];
    }

    for(int level = 0; level < nlevels; level++)
    {
        mvInvScaleFactor[level] = 1.0f/mvScaleFactor[level];
        //mvInvLevelSigma2[level] = 1.0f/mvLevelSigma2[level];
    }

    estimateDesiredNumberOfFeaturesPerLevel();

#ifdef ORB_FEATURE
    OrbFunctions::computeEndRowCircularPatch(pattern,umax);
#endif
}

void FeatureExtractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints)
{
    allKeypoints.resize(nlevels);

    const float W = 30;
    for (int level = 0; level < nlevels; ++level)
    {
        const int minBorderX = EDGE_THRESHOLD-3;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
        const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*10);

        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        for(int i=0; i<nRows; i++)
        {
            const float iniY =minBorderY+i*hCell;
            float maxY = iniY+hCell+6;

            if(iniY>=maxBorderY-3)
                continue;
            if(maxY>maxBorderY)
                maxY = maxBorderY;

            for(int j=0; j<nCols; j++)
            {
                const float iniX =minBorderX+j*wCell;
                float maxX = iniX+wCell+6;
                if(iniX>=maxBorderX-6)
                    continue;
                if(maxX>maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;
                FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                     vKeysCell,iniThFAST,true);

                if(vKeysCell.empty())
                {
                    FAST(mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,minThFAST,true);
                }

                if(!vKeysCell.empty())
                {
                    for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                    {
                        (*vit).pt.x+=j*wCell;
                        (*vit).pt.y+=i*hCell;
                        vToDistributeKeys.push_back(*vit);
                    }
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

        const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }
    }

    // compute orientations
    for (int level = 0; level < nlevels; ++level)
        OrbFunctions::computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
}

void FeatureExtractor::ComputeKeyPointsAndDescriptors(cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                                      std::vector<mat2>& keyPointsInformation)
{
    keypoints.clear();
    descriptors.release();
    keyPointsInformation.clear();

    cv::Ptr<DETECTOR_CV> detector;
    createDetector(detector);

    const int minBorderX = 0;
    const int minBorderY = 0;
    bool first{true};
    for(int level{0}; level < nlevels; level++){
        const int maxBorderX = mvImagePyramid[level].cols - 1;
        const int maxBorderY = mvImagePyramid[level].rows - 1;

        std::vector<cv::KeyPoint> keypointsTmp;
        detector->detect(mvImagePyramid[level], keypointsTmp);

        std::vector<cv::KeyPoint> distributedKeyPoints = DistributeOctTree(keypointsTmp, minBorderX, maxBorderX,
                                                minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);
        Mat descriptorsLevel{};
        detector->compute(mvImagePyramid[level], distributedKeyPoints, descriptorsLevel);

        for(auto& keypoint: distributedKeyPoints){
            keypoint.octave = level;
            keypoint.pt *= mvScaleFactor[level];
            keyPointsInformation.push_back(estimateKeyPointInformation(keypoint));
        }
        keypoints.insert(keypoints.end(), distributedKeyPoints.begin(), distributedKeyPoints.end());

        if(first){
            descriptors = descriptorsLevel.clone();
            first = false;
        }else{
            cv::vconcat(descriptors.clone(), descriptorsLevel, descriptors);
        }

    }
}

#ifdef ORB_FEATURE
void FeatureExtractor::operator()(InputArray _image, InputArray _mask,
        vector<KeyPoint>& _keypoints, OutputArray _descriptors, vector<mat2>& keyPointsInformation)
{
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // Pre-compute the scale pyramid
    ComputePyramid(image);

    vector < vector<KeyPoint> > allKeypoints;
    ComputeKeyPointsOctTree(allKeypoints);

    Mat descriptors;

    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();
    if( nkeypoints == 0 )
        _descriptors.release();
    else
    {
        _descriptors.create(nkeypoints, DESCRIPTOR_SIZE, DESCRIPTOR_MAT_TYPE);
        descriptors = _descriptors.getMat();
    }

    _keypoints.clear();
    _keypoints.reserve(nkeypoints);

    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkeypointsLevel = (int)keypoints.size();

        if(nkeypointsLevel==0)
            continue;

        // preprocess the resized image
        Mat workingMat = mvImagePyramid[level].clone();
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

        // Compute the descriptors
        Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
#ifdef ORB_FEATURE
        OrbFunctions::computeDescriptors(workingMat, keypoints, desc, pattern);
#endif
        offset += nkeypointsLevel;

        // Scale keypoint coordinates
        float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
        for (auto& keypoint: keypoints){
            keypoint.pt *= scale;
            keyPointsInformation.push_back(estimateKeyPointInformation(keypoint));
        }

        // And add the keypoints to the output
        _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
    }
    //cout << "_keypoints = "<< _keypoints.size() << endl;
    //cout << "_descriptors = "<< _descriptors.size() << endl;
    //cout << "keyPointsInformation = "<< keyPointsInformation.size() << endl;

}
#else
void FeatureExtractor::operator()(cv::InputArray _image, cv::InputArray mask,
                                  std::vector<cv::KeyPoint>& _keypoints,
                                  cv::Mat& _descriptors, std::vector<mat2>& _keyPointsInformation){
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );
    ComputePyramid(image); // TODO THIS SHOULDNT BE NECESSARY
    ComputeKeyPointsAndDescriptors(image,_keypoints, _descriptors, _keyPointsInformation);
}
#endif

void FeatureExtractor::updateDetectorSettings(cv::Ptr<DETECTOR_CV>& detector, float factor){
    float threshold = 0.00005f;
#if defined(AKAZE16_FEATURE) || defined(AKAZE32_FEATURE) || defined(AKAZE61_FEATURE)
    detector->setThreshold(threshold/factor);
#endif
#ifdef KAZE_FEATURE
    detector->setThreshold(threshold/2.0f);
#endif
}

void FeatureExtractor::createDetector(cv::Ptr<DETECTOR_CV>& detector){
#if defined(AKAZE16_FEATURE) || defined(AKAZE32_FEATURE) || defined(AKAZE61_FEATURE)
    const float threshold = 0.0001f;
    const int numOctaves = 1;
    const int numDifussionLevels = 1;

    // Descriptor Size <= 486 = 162*3 - 3 channels
    #ifdef AKAZE61_FEATURE
    detector = DETECTOR_CV::create(cv::AKAZE::AKAZE::DESCRIPTOR_MLDB,0,3,
                                   threshold,numOctaves,numDifussionLevels,KAZE::DIFF_PM_G2);
    #else
    detector = DETECTOR_CV::create(cv::AKAZE::AKAZE::DESCRIPTOR_MLDB,DESCRIPTOR_SIZE*8,3,
                                   threshold,numOctaves,numDifussionLevels,KAZE::DIFF_PM_G2);
    #endif
#endif

#ifdef BRISK_FEATURE
    const int thresh = 30;
    const int octaves = 1;
    const float patternScale = 1.0f;
    detector = DETECTOR_CV::create(thresh,octaves,patternScale);
#endif

#ifdef KAZE_FEATURE
    float threshold = 0.001f;
    detector->setThreshold(threshold);
    detector->setNOctaves(2);
    detector->setNOctaveLayers(4);
#endif

#ifdef SURF_FEATURE
    float threshold = 0.001f;
    //detector->setThreshold(threshold);
    detector->setNOctaves(2);
    detector->setNOctaveLayers(4);
#endif
}

void FeatureExtractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);            
        }
        else
        {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);            
        }
    }

}

mat2 FeatureExtractor::estimateKeyPointInformation(cv::KeyPoint& pt){
    mat2 information{mat2::Identity()};
    information(0,0) = 1.0/(mvScaleFactor[pt.octave]*mvScaleFactor[pt.octave]);
    information(1,1) = information(0,0);
    return information;
}

void FeatureExtractor::estimateDesiredNumberOfFeaturesPerLevel(){
    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);
}

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
        const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x+halfX,UL.y);
        n1.BL = cv::Point2i(UL.x,UL.y+halfY);
        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x,UL.y+halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x,BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for(size_t i=0;i<vKeys.size();i++)
        {
            const cv::KeyPoint &kp = vKeys[i];
            if(kp.pt.x<n1.UR.x)
            {
                if(kp.pt.y<n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if(kp.pt.y<n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if(n1.vKeys.size()==1)
            n1.bNoMore = true;
        if(n2.vKeys.size()==1)
            n2.bNoMore = true;
        if(n3.vKeys.size()==1)
            n3.bNoMore = true;
        if(n4.vKeys.size()==1)
            n4.bNoMore = true;

    }

vector<cv::KeyPoint> FeatureExtractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                             const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
{
    // Compute how many initial nodes
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    const float hX = static_cast<float>(maxX-minX)/nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

        for(int i=0; i<nIni; i++)
        {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
            ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
            ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
            ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for(size_t i=0;i<vToDistributeKeys.size();i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
        }

        list<ExtractorNode>::iterator lit = lNodes.begin();

        while(lit!=lNodes.end())
        {
            if(lit->vKeys.size()==1)
            {
                lit->bNoMore=true;
                lit++;
            }
            else if(lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size()*4);

        while(!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while(lit!=lNodes.end())
            {
                if(lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNode n1,n2,n3,n4;
                    lit->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit=lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
            {
                bFinish = true;
            }
            else if(((int)lNodes.size()+nToExpand*3)>N)
            {

                while(!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end(), cornerComparison);
                    for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                    {
                        ExtractorNode n1,n2,n3,n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                        // Add childs if they contain points
                        if(n1.vKeys.size()>0)
                        {
                            lNodes.push_front(n1);
                            if(n1.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n2.vKeys.size()>0)
                        {
                            lNodes.push_front(n2);
                            if(n2.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n3.vKeys.size()>0)
                        {
                            lNodes.push_front(n3);
                            if(n3.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n4.vKeys.size()>0)
                        {
                            lNodes.push_front(n4);
                            if(n4.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if((int)lNodes.size()>=N)
                            break;
                    }

                    if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint* pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            for(size_t k=1;k<vNodeKeys.size();k++)
            {
                if(vNodeKeys[k].response>maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }

} //namespace ORB_SLAM
