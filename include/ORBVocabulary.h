/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include "Definitions.h"

#ifdef COMPILED_WITH_DBOW2
#include DBOW_SRC_F
#include DBOW_SRC_TEMPLATEDVOCABULARY
#endif

#ifdef COMPILED_WITH_DBOW3
#include DBOW_SRC_DBOW3
#endif

namespace ORB_SLAM2
{

#ifdef COMPILED_WITH_DBOW2
    typedef DBOW::TemplatedVocabulary<DBOW::DESCRIPTOR_F::TDescriptor, DBOW::DESCRIPTOR_F> FEATUREVocabulary;
#endif

#ifdef COMPILED_WITH_DBOW3
    typedef DBOW::Vocabulary FEATUREVocabulary;
#endif

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
