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
#include "PENormalisation.h"

namespace PE
{
class Mock_normalisation: public normalisation
{

public:
    Mock_normalisation()
    {}
    virtual ~Mock_normalisation()
    {}
    MOCK_METHOD1(add_sensor, void(const TValue&));
};

} //namespace PE
#endif //__mock__PE_Normalisation_H__
