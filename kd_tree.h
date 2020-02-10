#pragma once

#include <algorithm>
#include <cassert>
#include <numeric>
#include <unordered_map>
#include <utility>

#include <visionaray/math/math.h>
#include <visionaray/aligned_vector.h>

#include <common/model.h>

#include "svt.h"

#define VERBOSE 0

namespace visionaray
{

class VSNRAY_ALIGN(16) kd_tree_node
{
public:

    aabb bbox;
    unsigned leaf_bit;

    struct index_range
    {
        unsigned first;
        unsigned last;
    };

    bool is_inner() const { return axis >> 30 != 3; }
    bool is_leaf() const { return axis >> 30 == 3; }

    float get_split() const
    {
        assert(is_inner());
        return split;
    }

    cartesian_axis<3> get_axis()
    {
        assert(is_inner());
        return cartesian_axis<3>::label(axis >> 30);
    }

    unsigned get_child(unsigned i = 0) const
    {
        assert(is_inner());
        return (first_child & 0x3FFFFFFF) + i;
    }

    index_range get_indices() const
    {
        assert(is_leaf());
        return { first_prim, first_prim + (num_prims & 0x3FFFFFFF) };
    }

    VSNRAY_FUNC unsigned get_first_primitive() const
    {
        assert(is_leaf());
        return first_prim;
    }

    VSNRAY_FUNC unsigned get_num_primitives() const
    {
        assert(is_leaf());
        return num_prims & 0x3FFFFFFF;
    }

    void set_inner(unsigned axis, float split)
    {
        assert(axis >= 0 && axis < 3);
        this->axis = axis << 30;
        this->split = split;
    }

    void set_leaf(unsigned first_primitive_index, unsigned count)
    {
        axis = 3 << 30;
        first_prim = first_primitive_index;
        num_prims |= count;
    }

    void set_first_child(unsigned index)
    {
        first_child |= index;
    }

private:
    union
    {
        float split;
        unsigned first_prim;
    };

    union
    {
        // AA--.----.----.----
        unsigned axis;

        // --NP.NPNP.NPNP.NPNP
        unsigned num_prims;

        // --FC.FCFC.FCFC.FCFC
        unsigned first_child;
    };
};

template <typename Primitive>
struct kd_tree
{
    float c1;
    float c2;
    float c3;
    float c4;
    Primitive /*const*/* primitives_;
    Instance* inst_data_;
    int num_primitives_;

    int num_meshes_; // meshes, not instances!

    std::vector<int> indices_;
    std::vector<aabb> prim_bounds_;
    std::vector<vec3> centroids_;

    aligned_vector<kd_tree_node> nodes_;

    // TODO: only temporarily
    aligned_vector<kd_tree_node> leaves_;

    aabb bbox_;

    SVT<unsigned> const& large_meshes_;

    // Map from first primitive index to MPI rank
    std::unordered_map<int, int> first_prim_to_rank_;

    unsigned leaf_bit = 1;

    // bitfields that indicate which leaves the primitive is assigned to
    std::vector<unsigned> primitives_to_leaves_;

    kd_tree(SVT<unsigned> const& large_meshes, float cc1, float cc2, float cc3, float cc4)
        : large_meshes_(large_meshes)
        , c1(cc1)
        , c2(cc2)
        , c3(cc3)
        , c4(cc4)
    {
    }

    kd_tree_node const& node(size_t index) const
    {
        return nodes_[index];
    }

    Primitive const& primitive(size_t index) const
    {
        return primitives_[indices_[index]];
    }

    // Return the comm rank given a *leaf* node index
    int rank_from_leaf_index(int index) const
    {
        auto& n = nodes_[index];
        assert(n.is_leaf());

        auto it = first_prim_to_rank_.find(static_cast<int>(n.get_first_primitive()));
        assert(it != first_prim_to_rank_.end());

        return it->second;
    }

    void init(
        Primitive /*const*/* primitives, // the instance BVHs
        int num_primitives,
        Instance* instData, // meta data with mapping from inst to mesh
        int num_meshes,
        int num_leaves
        )
    {
        primitives_ = primitives;
        inst_data_ = instData;
        num_primitives_ = num_primitives;
        num_meshes_ = num_meshes;

        primitives_to_leaves_.resize(num_primitives);
        std::fill(primitives_to_leaves_.begin(),
                  primitives_to_leaves_.end(),
                  0);

        // Initially, domain bounds = object bounds
        bbox_.invalidate();

        prim_bounds_.resize(num_primitives_);
        centroids_.resize(num_primitives_);

        for (int i = 0; i < num_primitives_; ++i)
        {
            auto bounds = get_bounds(primitives_[i]);

            bbox_.insert(bounds.min);
            bbox_.insert(bounds.max);

            prim_bounds_[i] = bounds;
            centroids_[i] = bounds.center();
        }std::cout << "object bounds: " << bbox_.min << bbox_.max << '\n';

        // Combine instance bounds with single-base mesh instances
        // out that we treated separately and aren't part of the
        // primitive/instance array
        bbox_.insert(large_meshes_.world_bounds);std::cout << "lm bounds: " << large_meshes_.world_bounds.min << large_meshes_.world_bounds.max << '\n';

        std::vector<int> indices_tmp(num_primitives_);
        std::iota(indices_tmp.begin(), indices_tmp.end(), 0);

        nodes_.emplace_back();

        do_split(
            0,
            indices_tmp,
            bbox_,
            num_leaves
            );

        int rank = 0;
        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            auto& n = nodes_[i];
            if (n.is_leaf())
            {
                first_prim_to_rank_.insert(
                        std::make_pair(n.get_first_primitive(), rank++)
                        );
            }
        }
    }

    void do_split(
            int index,
            std::vector<int> const& indices_in,
            aabb domain_bounds,
            int num_leaves
            )
    {
        if (num_leaves == 1)
        {
            auto& node = nodes_[index];

            node.leaf_bit = leaf_bit;

            unsigned first_index = indices_.size();

            node.bbox = domain_bounds;

            for (auto it = indices_in.begin(); it != indices_in.end(); ++it)
            {
                primitives_to_leaves_[*it] |= leaf_bit;

                indices_.push_back(*it);
            }

            unsigned count = indices_.size() - first_index;

            node.set_leaf(first_index, count);

            // TODO: only temporarily
            leaves_.emplace_back(node);

            leaf_bit <<= 1;
        }
        else
        {
            auto first_child_index = static_cast<int>(nodes_.size());

            std::vector<int> indices_left;
            std::vector<int> indices_right;

            aabb domain_boundsL;
            aabb domain_boundsR;

            nodes_[index] = split(
                indices_in,
                indices_left,
                indices_right,
                domain_bounds,
                domain_boundsL,
                domain_boundsR
                );

            nodes_[index].set_first_child(first_child_index);

            nodes_.emplace_back();
            nodes_.emplace_back();

            do_split(
                first_child_index,
                indices_left,
                domain_boundsL,
                num_leaves >> 1
                );

            do_split(
                first_child_index + 1,
                indices_right,
                domain_boundsR,
                num_leaves >> 1
                );
        }
    }

    kd_tree_node split(
            std::vector<int> const& indices_in,
            std::vector<int>& indices_left,
            std::vector<int>& indices_right,
            aabb domain_bounds,
            aabb& domain_boundsL,
            aabb& domain_boundsR
            )
    {
#if VERBOSE
        std::cout << "Coming in: " << indices_in.size() << " indices!\n";
#endif

        // Optimization (courtesy IW):
        // Clip the domain bounds with the object bounds,
        // hoping we can further reduce their size
        aabb object_bounds;
        object_bounds.invalidate();

        for (size_t i = 0; i < indices_in.size(); ++i)
        {
            auto bounds = get_bounds(primitives_[indices_in[i]]);

            object_bounds.insert(bounds.min);
            object_bounds.insert(bounds.max);
        }

        // Must also consider large meshes
        unsigned ignore;
        aabb large_mesh_bounds = large_meshes_.boundary(domain_bounds, ignore);
        object_bounds.insert(large_mesh_bounds);
#if VERBOSE
        std::cout << "OB: " << object_bounds.min << object_bounds.max << '\n';
#endif
        domain_bounds = intersect(domain_bounds, object_bounds);


        // Split domain bounds along their longest axis

        unsigned axis = domain_bounds.size()[0] > domain_bounds.size()[1] ? 0 : 1;
        axis = domain_bounds.size()[axis] > domain_bounds.size()[2] ? axis : 2;
#if VERBOSE
        std::cout << axis << '\n';
#endif


        std::vector<int> indices_temp(indices_in);

        // Calculate data for normalization!
        uint64_t total_num_triangles = 0;
        {
            std::vector<bool> used(num_meshes_);
            std::fill(used.begin(), used.end(), false);

            for (size_t i = 0; i < indices_temp.size(); ++i)
            {
                auto& inst = primitives_[indices_temp[i]];

                int mesh_index = inst_data_[indices_temp[i]].index;
                if (!used[mesh_index])
                {
                    total_num_triangles += inst.num_primitives();
                    used[mesh_index] = true;
                }
            }
        }

        unsigned fromLargeMeshes = 0;
        large_meshes_.boundary(domain_bounds, fromLargeMeshes);
        total_num_triangles += fromLargeMeshes/3;

        float split_pos = FLT_MAX;

        // costs
        uint64_t num_distributed_triangles(-1);
        uint64_t balance(-1);
        float best_median_split_dist = .0f; // extra costs for distance to median split

        float best_costs = FLT_MAX;

        static const int Bins = 64;
        float bin_width = domain_bounds.size()[axis] / Bins;

        for (int b = 1; b <= Bins-1; ++b)
        {
            float sp = domain_bounds.min[axis] + bin_width * b;

            uint64_t num_triangles_left = 0;
            uint64_t num_triangles_right = 0;

            std::vector<bool> used_left(num_meshes_);
            std::vector<bool> used_right(num_meshes_);
            std::fill(used_left.begin(), used_left.end(), false);
            std::fill(used_right.begin(), used_right.end(), false);

            for (size_t i = 0; i < indices_temp.size(); ++i)
            {
                int index = indices_temp[i];

                auto bounds = prim_bounds_[index];

                bool pushed_left = false;
                bool pushed_right = false;

                if (bounds.max[axis] < sp)
                {
                    //il.push_back(index);
                    pushed_left = true;
                }
                else if (bounds.min[axis] >= sp)
                {
                    //ir.push_back(index);
                    pushed_right = true;
                }
                else
                {
                    //il.push_back(index);
                    //ir.push_back(index);
                    pushed_left = true;
                    pushed_right = true;
                }

                if (pushed_left)
                {
                    auto& inst = primitives_[index];
                    int mesh_index = inst_data_[index].index;

                    if (!used_left[mesh_index])
                    {
                        num_triangles_left += inst.num_primitives();
                        used_left[mesh_index] = true;
                    }
                }

                if (pushed_right)
                {
                    auto& inst = primitives_[index];
                    int mesh_index = inst_data_[index].index;

                    if (!used_right[mesh_index])
                    {
                        num_triangles_right += inst.num_primitives();
                        used_right[mesh_index] = true;
                    }
                }

                //il.clear();
                //ir.clear();
            }

            // Handle large meshes!

            // That's the child domain bounds we'd obtain from these splits
            auto child_domain_bounds = visionaray::split(domain_bounds, cartesian_axis<3>::label(axis), sp);
            //std::cout << "P: " << domain_bounds.min << domain_bounds.max << '\n';
            //std::cout << "Split at: " << axis << ' ' << sp << '\n';
            //std::cout << "L: " << child_domain_bounds.first.min << child_domain_bounds.first.max << '\n';
            //std::cout << "R: " << child_domain_bounds.second.min << child_domain_bounds.second.max << '\n';
            //std::cout << '\n';

            // count the number of vertices from large meshes
            // to the left and the right of the split plane
            unsigned lm_vertices_left = 0;
            large_meshes_.boundary(child_domain_bounds.first, lm_vertices_left);
            unsigned lm_vertices_right = 0;
            large_meshes_.boundary(child_domain_bounds.second, lm_vertices_right);
#if VERBOSE
            std::cout << "Large mesh triangles L and R " << (lm_vertices_left/3) << ' ' << (lm_vertices_right/3) << '\n';
#endif
            num_triangles_left += lm_vertices_left/3;
            num_triangles_right += lm_vertices_right/3;


            uint64_t num_triangles = num_triangles_left + num_triangles_right;
            auto dist = std::abs(int64_t(num_triangles_left - num_triangles_right));

            // How far are we from median split?
            float median_split_pos = .0f;//FLT_MAX;
            if (indices_temp.size() > 0)
                median_split_pos = get_bounds(primitives_[indices_temp[indices_temp.size()/2]]).center()[axis];
            float median_split_dist = std::abs(median_split_pos - sp);

            // How far are we from middle split
            float middle_split_pos = (domain_bounds.min[axis] + domain_bounds.max[axis]) / 2.f;
            float middle_split_dist = std::abs(middle_split_pos - sp);

            // Normalize all:
            float num_triangles_n = (float)num_triangles / (total_num_triangles*2);
            float dist_n = (float)dist / total_num_triangles;
            float median_split_dist_n = median_split_dist / object_bounds.size()[axis];
            float middle_split_dist_n = middle_split_dist / (object_bounds.size()[axis]/2);

            float costs = c1 * num_triangles_n + c2 * dist_n + c3 * median_split_dist_n + c4 * middle_split_dist_n;

#if 0//VERBOSE
            std::cout << num_triangles_n  << ' ' << dist_n << ' ' << median_split_dist_n << ' ' << middle_split_dist_n << '\n';
            std::cout << "c1: " << c1 * num_triangles_n << '\n';
            std::cout << "c2: " << c2 * dist_n << '\n';
            std::cout << "c3: " << c3 * median_split_dist_n << '\n';
            std::cout << "c4: " << c4 * middle_split_dist_n << '\n';
            std::cout << "Costs: " << costs << '\n';
#endif

            if (costs < best_costs)
            {
#if 0//VERBOSE
                std::cout << "#triangles: " << num_triangles_left << ' ' << num_triangles_right << '\n';
#endif
                best_costs = costs;
                split_pos = sp;

                domain_boundsL = child_domain_bounds.first;
                domain_boundsR = child_domain_bounds.second;
            }
        }

        // Fill index arrays based on the split we found
        static int l=0,r=0,b=0;
        for (size_t i = 0; i < indices_temp.size(); ++i)
        {
            auto bounds = prim_bounds_[indices_temp[i]];

            if (bounds.max[axis] <= split_pos)
            {++l;
                indices_left.push_back(indices_temp[i]);
            }
            else if (bounds.min[axis] >= split_pos)
            {++r;
                indices_right.push_back(indices_temp[i]);
            }
            else
            {++b;
                indices_left.push_back(indices_temp[i]);
                indices_right.push_back(indices_temp[i]);
            }
        }
#if VERBOSE
        std::cout << l << ' ' << r << ' ' << b << '\n';
#endif

        kd_tree_node result;
        result.set_inner(axis, split_pos);
        return result;
    }
};

} // visionaray
