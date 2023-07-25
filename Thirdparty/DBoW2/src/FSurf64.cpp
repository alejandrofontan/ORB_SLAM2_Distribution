/**
 * File: FSurf64.cpp
 * Date: July 2023
 * Author: Alejandro Fontan Villacampa
 * Description: functions for Surf64 descriptors
 * License: see the LICENSE.txt file
 *
 */
 
#include <vector>
#include <string>
#include <sstream>

#include "FClass.h"
#include "FSurf64.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void DESCRIPTOR_CLASS_NAME::meanValue(const std::vector<DESCRIPTOR_CLASS_NAME::pDescriptor> &descriptors, DESCRIPTOR_CLASS_NAME::TDescriptor &mean)
{
  mean.resize(0);
  mean.resize(DESCRIPTOR_CLASS_NAME::L, 0);
  
  float s = descriptors.size();
  
  vector<DESCRIPTOR_CLASS_NAME::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
    const DESCRIPTOR_CLASS_NAME::TDescriptor &desc = **it;
    for(int i = 0; i < DESCRIPTOR_CLASS_NAME::L; i += 4)
    {
      mean[i  ] += desc[i  ] / s;
      mean[i+1] += desc[i+1] / s;
      mean[i+2] += desc[i+2] / s;
      mean[i+3] += desc[i+3] / s;
    }
  }
}

// --------------------------------------------------------------------------
  
double DESCRIPTOR_CLASS_NAME::distance(const DESCRIPTOR_CLASS_NAME::TDescriptor &a, const DESCRIPTOR_CLASS_NAME::TDescriptor &b)
{
  double sqd = 0.;
  for(int i = 0; i < DESCRIPTOR_CLASS_NAME::L; i += 4)
  {
    sqd += (a[i  ] - b[i  ])*(a[i  ] - b[i  ]);
    sqd += (a[i+1] - b[i+1])*(a[i+1] - b[i+1]);
    sqd += (a[i+2] - b[i+2])*(a[i+2] - b[i+2]);
    sqd += (a[i+3] - b[i+3])*(a[i+3] - b[i+3]);
  }
  return sqd;
}

// --------------------------------------------------------------------------

std::string DESCRIPTOR_CLASS_NAME::toString(const DESCRIPTOR_CLASS_NAME::TDescriptor &a)
{
  stringstream ss;
  for(int i = 0; i < DESCRIPTOR_CLASS_NAME::L; ++i)
  {
    ss << a[i] << " ";
  }
  return ss.str();
}

// --------------------------------------------------------------------------
  
void DESCRIPTOR_CLASS_NAME::fromString(DESCRIPTOR_CLASS_NAME::TDescriptor &a, const std::string &s)
{
  a.resize(DESCRIPTOR_CLASS_NAME::L);
  
  stringstream ss(s);
  for(int i = 0; i < DESCRIPTOR_CLASS_NAME::L; ++i)
  {
    ss >> a[i];
  }
}

// --------------------------------------------------------------------------

void DESCRIPTOR_CLASS_NAME::toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const int N = descriptors.size();
  const int L = DESCRIPTOR_CLASS_NAME::L;
  
  mat.create(N, L, CV_32F);
  
  for(int i = 0; i < N; ++i)
  {
    const TDescriptor& desc = descriptors[i];
    float *p = mat.ptr<float>(i);
    for(int j = 0; j < L; ++j, ++p)
    {
      *p = desc[j];
    }
  } 
}

// --------------------------------------------------------------------------

} // namespace DBoW2

