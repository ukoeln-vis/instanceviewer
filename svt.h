// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <algorithm>
#include <vector>

#include <visionaray/math/aabb.h>
#include <visionaray/math/forward.h>
#include <visionaray/math/matrix.h>
#include <visionaray/math/vector.h>

namespace visionaray
{

//-------------------------------------------------------------------------------------------------
// Summed-volume table
//

template <typename T>
struct SVT
{
    // Init from instances (should only have count 1!)
    template <typename I>
    void init(I const* instances, size_t num_instances, vec3i size);

    // Boundary, box is in svt coords
    aabbi boundary(aabbi bbox, T& count) const;

    // Boundary, box is in world coords
    aabb boundary(aabb bbox, T& count) const;

  T& operator()(int x, int y, int z)
  {
    return data_[z * width * height + y * width + x];
  }

  T& at(int x, int y, int z)
  {
    return data_[z * width * height + y * width + x];
  }

  T const& at(int x, int y, int z) const
  {
    return data_[z * width * height + y * width + x];
  }

  T border_at(int x, int y, int z) const
  {
    if (x < 0 || y < 0 || z < 0)
      return 0;

    return data_[z * width * height + y * width + x];
  }

  T last() const
  {
    return data_.back();
  }

  T* data()
  {
    return data_.data();
  }

  T const* data() const
  {
    return data_.data();
  }

  T get_count(aabbi bounds) const
  {
    bounds.min -= vec3i(1);
    bounds.max -= vec3i(1);

    return border_at(bounds.max.x, bounds.max.y, bounds.max.z)
         - border_at(bounds.max.x, bounds.max.y, bounds.min.z)
         - border_at(bounds.max.x, bounds.min.y, bounds.max.z)
         - border_at(bounds.min.x, bounds.max.y, bounds.max.z)
         + border_at(bounds.min.x, bounds.min.y, bounds.max.z)
         + border_at(bounds.min.x, bounds.max.y, bounds.min.z)
         + border_at(bounds.max.x, bounds.min.y, bounds.min.z)
         - border_at(bounds.min.x, bounds.min.y, bounds.min.z);
  }

    vec3 to_world_coords(vec3i v) const
    {
        assert(world_bounds.valid());

        vec3 vf(v);

        vec3 sizef(width, height, depth);
        sizef -= vec3(1.f);
        sizef += vec3(.999f); // e.g. 255.999

        vf /= sizef;

        vf *= world_bounds.size();
        vf += world_bounds.min;

        return vf;
    }

    vec3i to_svt_coords(vec3 v) const
    {
        assert(world_bounds.valid());

        v -= world_bounds.min;
        v /= world_bounds.size();

        vec3 sizef(width, height, depth);
        sizef -= vec3(1.f);
        sizef += vec3(.999f); // e.g. 255.999

        v *= sizef;

        return vec3i(v);
    }

    // SVT array
    std::vector<T> data_;
    int width;
    int height;
    int depth;

    aabb world_bounds;
};

template <typename T>
template <typename I>
void SVT<T>::init(I const* instances, size_t num_instances, vec3i size)
{
    data_.resize(size.x * size.y * size.z);
    std::fill(data_.begin(), data_.end(), 0);
    width  = size.x;
    height = size.y;
    depth  = size.z;

    if (num_instances == 0)
    {
        return;
    }

    // Build world bounds - for better precision:
    // use transformed vertices instead of transformed loca bounds though

    world_bounds.invalidate();

    for (size_t i = 0; i < num_instances; ++i)
    {
        auto& inst = instances[i];

        mat4 trans = inverse(inst.transform_inv());

        for (size_t j = 0; j < inst.num_primitives(); ++j)
        {
            auto tri = inst.get_ref().primitives_first[j];

            vec3 v1 = tri.v1;
            vec3 v2 = tri.v1 + tri.e1;
            vec3 v3 = tri.v1 + tri.e2;

            v1 = (trans * vec4(v1, 1.0f)).xyz();
            v2 = (trans * vec4(v2, 1.0f)).xyz();
            v3 = (trans * vec4(v3, 1.0f)).xyz();

            world_bounds.insert(v1);
            world_bounds.insert(v2);
            world_bounds.insert(v3);
        }
    }

//  std::cout << world_bounds.min << world_bounds.max << '\n';

    // Convert to SVT coordinates, project into SVT
    for (size_t i = 0; i < num_instances; ++i)
    {
        auto& inst = instances[i];

        mat4 trans = inverse(inst.transform_inv());

        for (size_t j = 0; j < inst.num_primitives(); ++j)
        {
            auto tri = inst.get_ref().primitives_first[j];

            vec3 v1 = tri.v1;
            vec3 v2 = tri.v1 + tri.e1;
            vec3 v3 = tri.v1 + tri.e2;

            v1 = (trans * vec4(v1, 1.0f)).xyz();
            v2 = (trans * vec4(v2, 1.0f)).xyz();
            v3 = (trans * vec4(v3, 1.0f)).xyz();

            vec3i v1i = to_svt_coords(v1);
            vec3i v2i = to_svt_coords(v2);
            vec3i v3i = to_svt_coords(v3);

            size_t index1 = v1i.z * width * height + v1i.y * width + v1i.x;
            size_t index2 = v2i.z * width * height + v2i.y * width + v2i.x;
            size_t index3 = v3i.z * width * height + v3i.y * width + v3i.x;

            data_[index1]++;
            data_[index2]++;
            data_[index3]++;
        }
    }


  // Build summed volume table

  // Init 0-border voxel
  //at(0, 0, 0) = at(0, 0, 0);

  // Init 0-border edges (prefix sum)
  for (int x=1; x<width; ++x)
  {
    at(x, 0, 0) = at(x, 0, 0) + at(x-1, 0, 0);
  }

  for (int y=1; y<height; ++y)
  {
    at(0, y, 0) = at(0, y, 0) + at(0, y-1, 0);
  }

  for (int z=1; z<depth; ++z)
  {
    at(0, 0, z) = at(0, 0, z) + at(0, 0, z-1);
  }


  // Init 0-border planes (summed-area tables)
  for (int y=1; y<height; ++y)
  {
    for (int x=1; x<width; ++x)
    {
      at(x, y, 0) = at(x, y, 0)
        + at(x-1, y, 0) + at(x, y-1, 0)
        - at(x-1, y-1, 0);
    }
  }

  for (int z=1; z<depth; ++z)
  {
    for (int y=1; y<height; ++y)
    {
      at(0, y, z) = at(0, y, z)
        + at(0, y-1, z) + at(0, y, z-1)
        - at(0, y-1, z-1);
    }
  }

  for (int x=1; x<width; ++x)
  {
    for (int z=1; z<depth; ++z)
    {
      at(x, 0, z) = at(x, 0, z)
        + at(x-1, 0, z) + at(x, 0, z-1)
        - at(x-1, 0, z-1);
    }
  }


  // Build up SVT
  for (int z=1; z<depth; ++z)
  {
    for (int y=1; y<height; ++y)
    {
      for (int x=1; x<width; ++x)
      {
        at(x, y, z) = at(x, y, z) + at(x-1, y-1, z-1)
          + at(x-1, y, z) - at(x, y-1, z-1)
          + at(x, y-1, z) - at(x-1, y, z-1)
          + at(x, y, z-1) - at(x-1, y-1, z);
      }
    }
  }
}

// produce a boundary around the *non-empty* voxels in bbox
template <typename T>
aabbi SVT<T>::boundary(aabbi bbox, T& count) const
{
  aabbi bounds = bbox;

  // Search for the minimal volume bounding box
  // that contains #voxels contained in bbox!
  count = get_count(bounds);


  // X boundary
  int x = (bounds.max.x - bounds.min.x) / 2;

  while (x >= 1)
  {
    aabbi lbox = bounds;
    lbox.min.x += x;

    if (get_count(lbox) == count)
    {
      bounds = lbox;
    }

    aabbi rbox = bounds;
    rbox.max.x -= x;

    if (get_count(rbox) == count)
    {
      bounds = rbox;
    }

    x /= 2;
  }

  // Y boundary from left
  int y = (bounds.max.y - bounds.min.y) / 2;

  while (y >= 1)
  {
    aabbi lbox = bounds;
    lbox.min.y += y;

    if (get_count(lbox) == count)
    {
      bounds = lbox;
    }

    aabbi rbox = bounds;
    rbox.max.y -= y;

    if (get_count(rbox) == count)
    {
      bounds = rbox;
    }

    y /= 2;
  }

  // Z boundary from left
  int z = (bounds.max.z - bounds.min.z) / 2;

  while (z >= 1)
  {
    aabbi lbox = bounds;
    lbox.min.z += z;

    if (get_count(lbox) == count)
    {
      bounds = lbox;
    }

    aabbi rbox = bounds;
    rbox.max.z -= z;

    if (get_count(rbox) == count)
    {
      bounds = rbox;
    }

    z /= 2;
  }

  return bounds;
}

// produce a boundary around the *non-empty* voxels in bbox
// bbox is in world coords!
template <typename T>
aabb SVT<T>::boundary(aabb bbox, T& count) const
{
    vec3i bmin = to_svt_coords(bbox.min);
    vec3i bmax = to_svt_coords(bbox.max);

    bmin = clamp(bmin, vec3i(0), vec3i(width, height, depth));
    bmax = clamp(bmax, vec3i(0), vec3i(width, height, depth));

    aabbi bbox_svt(bmin, bmax);

    aabbi boundary_svt = boundary(bbox_svt, count);

    return aabb(to_world_coords(boundary_svt.min), to_world_coords(boundary_svt.max));
}

} // visionaray
