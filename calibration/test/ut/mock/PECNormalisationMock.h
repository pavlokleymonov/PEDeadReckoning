/**
 * Position Engine provides dead reckoning engine to obtain position
 * information based on fusion of different kind of sensors.
 *
 * Copyright 2017 Pavlo Kleymonov <pavlo.kleymonov@gmail.com>
 *
 * Distributed under the OSI-approved BSD License (the "License");
 * see accompanying file LICENSE.txt for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */
#ifndef __mock__PE_Normalisation_H__
#define __mock__PE_Normalisation_H__

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "PECNormalisation.h"

namespace PE
{
class CNormalisationMock: public CNormalisation
{

public:
    CNormalisationMock()
    {}
    virtual ~CNormalisationMock()
    {}
    MOCK_METHOD1(AddSensor, void(const TValue&));
};

} //namespace PE
#endif //__mock__PE_Normalisation_H__
