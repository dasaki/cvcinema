/****************************************************************************
*
* AdvancedBGS.hpp
*
* Purpose: Header to be included where you need to acces the BGS algorithms
*
* Author: David Sanz Kirbis, January 2011
*
* Comments: to be used with Donovan Parks 2007 BGS source code found at
*           http://dparks.wikidot.com/source-code
*
*
******************************************************************************/

#ifndef AdvancedBGS_H_
#define AdvancedBGS_H_


#include "AdaptiveMedianBGS.hpp"
#include "Eigenbackground.hpp"
#include "GrimsonGMM.hpp"
#include "MeanBGS.hpp"
#include "PratiMediodBGS.hpp"
#include "WrenGA.hpp"
#include "ZivkovicAGMM.hpp"

		struct bgs_struct {
            Algorithms::BackgroundSubtraction::Bgs          *bgsPtr;
            Algorithms::BackgroundSubtraction::BgsParams    *paramsPtr;
            int                                             On;
            string                                          name;
            BwImage                                         low;
            BwImage                                         high;
        };

#endif
