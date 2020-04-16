// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <visionaray/math/forward.h>
#include <visionaray/math/matrix.h>

//-------------------------------------------------------------------------------------------------
// Instance
//

namespace visionaray
{

struct Instance
{
    int index;
    mat4 transform;
};

} // visionaray
