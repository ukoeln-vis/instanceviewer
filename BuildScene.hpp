// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

//-------------------------------------------------------------------------------------------------
// Ignore this file, it's very verbose and only there to transform the loaded pbrt/json scene
// to a format that is compatible with Visionaray
//

#include <visionaray/math/math.h>
#include <visionaray/texture/texture.h>
#include <visionaray/aligned_vector.h>
#include <visionaray/area_light.h>
#include <visionaray/bvh.h>
#include <visionaray/environment_light.h>
#include <visionaray/generic_light.h>
#include <visionaray/generic_material.h>
#include <visionaray/point_light.h>
#include <visionaray/spot_light.h>

#include <common/config.h>

#include <common/image.h>
#include <common/make_materials.h>
#include <common/make_texture.h>
#include <common/model.h>
#include <common/sg.h>

#if VSNRAY_COMMON_HAVE_PTEX
#include <common/ptex.h>
#endif

#include "Instance.hpp"

enum class TexFormat { UV, Ptex };

namespace visionaray
{

using bvh_type = index_bvh<basic_triangle<3, float>>;
using generic_light_t = generic_light<
        point_light<float>,
        spot_light<float>,
        area_light<float, basic_triangle<3, float>>//,
        //area_light<float, basic_sphere<float>>
        >;
using generic_material_t = generic_material<
    emissive<float>,
    glass<float>,
    matte<float>,
    metal<float>,
    mirror<float>,
    plastic<float>
    >;


//-------------------------------------------------------------------------------------------------
// Map obj material to generic material
//

generic_material_t map_material(sg::obj_material const& mat)
{
    // Add emissive material if emissive component > 0
    if (length(mat.ce) > 0.0f)
    {
        emissive<float> em;
        em.ce() = from_rgb(mat.ce);
        em.ls() = 1.0f;
        return em;
    }
    else if (mat.illum == 1)
    {
        matte<float> ma;
        ma.ca() = from_rgb(mat.ca);
        ma.cd() = from_rgb(mat.cd);
        ma.ka() = 1.0f;
        ma.kd() = 1.0f;
        return ma;
    }
    else if (mat.illum == 3)
    {
        mirror<float> mi;
        mi.cr() = from_rgb(mat.cs);
        mi.kr() = 1.0f;
        mi.ior() = spectrum<float>(0.0f);
        mi.absorption() = spectrum<float>(0.0f);
        return mi;
    }
    else if (mat.illum == 4 && mat.transmission > 0.0f)
    {
        glass<float> gl;
        gl.ct() = from_rgb(mat.cd);
        gl.kt() = 1.0f;
        gl.cr() = from_rgb(mat.cs);
        gl.kr() = 1.0f;
        gl.ior() = from_rgb(mat.ior);
        return gl;
    }
    else
    {
        plastic<float> pl;
        pl.ca() = from_rgb(mat.ca);
        pl.cd() = from_rgb(mat.cd);
        pl.cs() = from_rgb(mat.cs);
        pl.ka() = 1.0f;
        pl.kd() = 1.0f;
        pl.ks() = 1.0f;
        pl.specular_exp() = mat.specular_exp;
        return pl;
    }
}


//-------------------------------------------------------------------------------------------------
// Map disney material to generic material
//

generic_material_t map_material(sg::disney_material const& mat)
{
    // TODO..
    if (mat.refractive > 0.0f)
    {
        glass<float> gl;
        gl.ct() = from_rgb(mat.base_color.xyz());
        gl.kt() = 1.0f;
        gl.cr() = from_rgb(vec3(mat.spec_trans));
        gl.kr() = 1.0f;
        gl.ior() = from_rgb(vec3(mat.ior));
        return gl;
    }
    else
    {
        matte<float> ma;
        ma.ca() = from_rgb(vec3(0.0f));
        ma.cd() = from_rgb(vec3(mat.base_color.xyz()));
        ma.ka() = 1.0f;
        ma.kd() = 1.0f;
        return ma;
    }
}


//-------------------------------------------------------------------------------------------------
// Map metal material to generic material
//

generic_material_t map_material(sg::metal_material const& mat)
{
    // TODO: consolidate glass and sg::glass_material (?)
    metal<float> mt;
    mt.roughness() = mat.roughness;
    mt.absorption() = from_rgb(mat.absorption);
    mt.ior() = from_rgb(mat.ior);
    return mt;
}


//-------------------------------------------------------------------------------------------------
// Map glass material to generic material
//

generic_material_t map_material(sg::glass_material const& mat)
{
    // TODO: consolidate glass and sg::glass_material (?)
    glass<float> gl;
    gl.ct() = from_rgb(mat.ct);
    gl.kt() = 1.0f;
    gl.cr() = from_rgb(mat.cr);
    gl.kr() = 1.0f;
    gl.ior() = from_rgb(mat.ior);
    return gl;
}
//-------------------------------------------------------------------------------------------------
// Approximate sphere with icosahedron
// cf. https://schneide.blog/2016/07/15/generating-an-icosphere-in-c/
//

struct icosahedron
{
    aligned_vector<basic_triangle<3, float>> triangles;
    aligned_vector<vec3> normals;
};

icosahedron make_icosahedron()
{
    static constexpr float X = 0.525731112119133606f;
    static constexpr float Z = 0.850650808352039932f;
    static constexpr float N = 0.0f;

    static const vec3 vertices[] = {
        { -X,  N,  Z },
        {  X,  N,  Z },
        { -X,  N, -Z },
        {  X,  N, -Z },
        {  N,  Z,  X },
        {  N,  Z, -X },
        {  N, -Z,  X },
        {  N, -Z, -X },
        {  Z,  X,  N },
        { -Z,  X,  N },
        {  Z, -X,  N },
        { -Z, -X,  N }
        };

    static const vec3i indices[] {
        { 0, 4, 1 },
        { 0, 9, 4 },
        { 9, 5, 4 },
        { 4, 5, 8 },
        { 4, 8, 1 },
        { 8, 10, 1 },
        { 8, 3, 10 },
        { 5, 3, 8 },
        { 5, 2, 3 },
        { 2, 7, 3 },
        { 7, 10, 3 },
        { 7, 6, 10 },
        { 7, 11, 6 },
        { 11, 0, 6 },
        { 0, 1, 6 },
        { 6, 1, 10 },
        { 9, 0, 11 },
        { 9, 11, 2 },
        { 9, 2, 5 },
        { 7, 2, 11 }
        };

    auto make_triangle = [&](int index)
    {
        vec3i idx = indices[index];
        return basic_triangle<3, float>(
                vertices[idx.x],
                vertices[idx.y] - vertices[idx.x],
                vertices[idx.z] - vertices[idx.x]
                );
    };

    icosahedron result;
    result.triangles.resize(20);
    result.normals.resize(20 * 3);

    for (int i = 0; i < 20; ++i)
    {
        result.triangles[i] = make_triangle(i);

        vec3i idx = indices[i];

        vec3 v1 = vertices[idx.x];
        vec3 v2 = vertices[idx.y];
        vec3 v3 = vertices[idx.z];

        result.normals[i * 3] = normalize(v1);
        result.normals[i * 3 + 1] = normalize(v2);
        result.normals[i * 3 + 2] = normalize(v3);
    }

    return result;
}


//-------------------------------------------------------------------------------------------------
// Reset triangle mesh flags to 0
//

struct reset_flags_visitor : sg::node_visitor
{
    using node_visitor::apply;

    void apply(sg::surface_properties& sp)
    {
        sp.flags() = 0;

        node_visitor::apply(sp);
    }

    void apply(sg::sphere& sph)
    {
        sph.flags() = 0;

        node_visitor::apply(sph);
    }

    void apply(sg::triangle_mesh& tm)
    {
        tm.flags() = 0;

        node_visitor::apply(tm);
    }

    void apply(sg::indexed_triangle_mesh& itm)
    {
        itm.flags() = 0;

        node_visitor::apply(itm);
    }
};


//-------------------------------------------------------------------------------------------------
// Traverse the scene graph to construct geometry, materials and BVH instances
//

struct build_scene_visitor : sg::node_visitor
{
    using node_visitor::apply;

    using bvh_type = index_bvh<basic_triangle<3, float>>;

    build_scene_visitor(
            aligned_vector<bvh_type>& bvhs,
            aligned_vector<Instance>& instances,
            aligned_vector<vec3>& shading_normals,
            aligned_vector<vec3>& geometric_normals,
            aligned_vector<vec2>& tex_coords,
            aligned_vector<vec3>& colors,
#if VSNRAY_COMMON_HAVE_PTEX
            aligned_vector<ptex::face_id_t>& face_ids,
#endif
            aligned_vector<std::pair<std::string, thin_lens_camera>>& cameras,
            aligned_vector<point_light<float>>& point_lights,
            aligned_vector<spot_light<float>>& spot_lights,
            visionaray::texture<vec4, 2>& env_map,
            environment_light<float, texture_ref<vec4, 2>>& env_light
            )
        : bvhs_(bvhs)
        , instances_(instances)
        , shading_normals_(shading_normals)
        , geometric_normals_(geometric_normals)
        , tex_coords_(tex_coords)
        , colors_(colors)
#if VSNRAY_COMMON_HAVE_PTEX
        , face_ids_(face_ids)
#endif
        , cameras_(cameras)
        , point_lights_(point_lights)
        , spot_lights_(spot_lights)
        , env_map_(env_map)
        , env_light_(env_light)
    {
    }

    void apply(sg::camera& c)
    {
        cameras_.push_back(std::make_pair(c.name(), static_cast<thin_lens_camera>(c)));

        node_visitor::apply(c);
    }

    void apply(sg::point_light& pl)
    {
        point_lights_.push_back(static_cast<visionaray::point_light<float>>(pl));

        vec3 pos = point_lights_.back().position();
        pos = (current_transform_ * vec4(pos, 1.0f)).xyz();
        point_lights_.back().set_position(pos);

        node_visitor::apply(pl);
    }

    void apply(sg::spot_light& sl)
    {
        spot_lights_.push_back(static_cast<visionaray::spot_light<float>>(sl));

        vec3 pos = spot_lights_.back().position();
        pos = (current_transform_ * vec4(pos, 1.0f)).xyz();
        spot_lights_.back().set_position(pos);

        node_visitor::apply(sl);
    }

    void apply(sg::environment_light& el)
    {
        auto tex = std::dynamic_pointer_cast<sg::texture2d<vec4>>(el.texture());

        if (tex != nullptr)
        {
            env_map_ = visionaray::texture<vec4, 2>(tex->width(), tex->height());
            env_map_.set_address_mode(tex->get_address_mode());
            env_map_.set_filter_mode(tex->get_filter_mode());
            env_map_.reset(tex->data());

            env_light_.texture() = texture_ref<vec4, 2>(env_map_);
            env_light_.scale() = from_rgb(el.scale());
            env_light_.set_light_to_world_transform(el.light_to_world_transform());
        }

        node_visitor::apply(el);
    }

    void apply(sg::transform& t)
    {
        mat4 prev = current_transform_;

        current_transform_ = current_transform_ * t.matrix();

        node_visitor::apply(t);

        current_transform_ = prev;
    }

    void apply(sg::surface_properties& sp)
    {
        unsigned prev = current_geom_id_;

        if (sp.flags() == 0 && sp.material() && sp.textures().find("diffuse") != sp.textures().end())
        {
            std::shared_ptr<sg::material> material = sp.material();
            std::shared_ptr<sg::texture> texture = sp.textures()["diffuse"];

            auto surf = std::make_pair(material, texture);

            auto it = std::find(surfaces.begin(), surfaces.end(), surf);
            if (it == surfaces.end())
            {
                current_geom_id_ = static_cast<unsigned>(surfaces.size());
                surfaces.push_back(surf);
            }
            else
            {
                current_geom_id_ = static_cast<unsigned>(std::distance(surfaces.begin(), it));
            }

            sp.flags() = ~sp.flags();
        }

        node_visitor::apply(sp);

        current_geom_id_ = prev;
    }

    void apply(sg::sphere& sph)
    {
        if (sph.flags() == 0)
        {
            auto ico = make_icosahedron();

            shading_normals_.insert(shading_normals_.end(), ico.normals.begin(), ico.normals.end());

            for (size_t i = 0; i < ico.triangles.size(); ++i)
            {
                auto& tri = ico.triangles[i];

                tri.prim_id = current_prim_id_++;
                tri.geom_id = current_geom_id_;

                vec3 n = normalize(cross(tri.e1, tri.e2));

                geometric_normals_.emplace_back(n);
                tex_coords_.emplace_back(0.0f, 0.0f);
                tex_coords_.emplace_back(0.0f, 0.0f);
                tex_coords_.emplace_back(0.0f, 0.0f);
            }

            // Build single bvh
            if (0/*build_strategy_ == renderer::LBVH*/)
            {
                lbvh_builder builder;

                bvhs_.emplace_back(builder.build(bvh_type{}, ico.triangles.data(), ico.triangles.size()));
            }
            else
            {
                binned_sah_builder builder;
                builder.enable_spatial_splits(false/*build_strategy_ == renderer::Split*/);

                bvhs_.emplace_back(builder.build(bvh_type{}, ico.triangles.data(), ico.triangles.size()));
            }

            sph.flags() = ~(bvhs_.size() - 1);
        }

        instances_.push_back({ static_cast<int>(~sph.flags()), current_transform_ });

        node_visitor::apply(sph);
    }

    void apply(sg::triangle_mesh& tm)
    {
        if (tm.flags() == 0 && tm.vertices.size() > 0)
        {
            assert(tm.vertices.size() % 3 == 0);

            aligned_vector<basic_triangle<3, float>> triangles(tm.vertices.size() / 3);

            size_t first_geometric_normal = geometric_normals_.size();
            geometric_normals_.resize(geometric_normals_.size() + tm.vertices.size() / 3);

            shading_normals_.insert(shading_normals_.end(), tm.normals.begin(), tm.normals.end());

            tex_coords_.insert(tex_coords_.end(), tm.tex_coords.begin(), tm.tex_coords.end());

            size_t first_color = colors_.size();
            if (tm.colors.size() > 0)
            {
                colors_.resize(first_color + tm.colors.size());
                for (size_t i = 0; i < tm.colors.size(); ++i)
                {
                    colors_[first_color + i] = vec3(tm.colors[i]);
                }
            }

#if VSNRAY_COMMON_HAVE_PTEX
            face_ids_.insert(face_ids_.end(), tm.face_ids.begin(), tm.face_ids.end());
#endif

            for (size_t i = 0; i < tm.vertices.size(); i += 3)
            {
                vec3 v1 = tm.vertices[i];
                vec3 v2 = tm.vertices[i + 1];
                vec3 v3 = tm.vertices[i + 2];

                basic_triangle<3, float> tri(v1, v2 - v1, v3 - v1);
                tri.prim_id = current_prim_id_++;
                tri.geom_id = current_geom_id_;
                triangles[i / 3] = tri;

                vec3 gn = normalize(cross(v2 - v1, v3 - v1));

                geometric_normals_[first_geometric_normal + i / 3] = gn;
            }

            // Build single bvh
            if (0/*build_strategy_ == renderer::LBVH*/)
            {
                lbvh_builder builder;

                bvhs_.emplace_back(builder.build(bvh_type{}, triangles.data(), triangles.size()));
            }
            else
            {
                binned_sah_builder builder;
                builder.enable_spatial_splits(false/*build_strategy_ == renderer::Split*/);

                bvhs_.emplace_back(builder.build(bvh_type{}, triangles.data(), triangles.size()));
            }

            tm.flags() = ~(bvhs_.size() - 1);
        }

        instances_.push_back({ static_cast<int>(~tm.flags()), current_transform_ });

        node_visitor::apply(tm);
    }

    void apply(sg::indexed_triangle_mesh& itm)
    {
        if (itm.flags() == 0 && itm.vertex_indices.size() > 0)
        {
            assert(itm.vertex_indices.size() % 3 == 0);

            aligned_vector<basic_triangle<3, float>> triangles(itm.vertex_indices.size() / 3);

            size_t first_geometric_normal = geometric_normals_.size();
            geometric_normals_.resize(geometric_normals_.size() + itm.vertex_indices.size() / 3);

            size_t first_shading_normal = shading_normals_.size();
            if (itm.normal_indices.size() > 0)
            {
                assert(itm.normal_indices.size() % 3 == 0);
                shading_normals_.resize(shading_normals_.size() + itm.normal_indices.size());
            }
            else
            {
                shading_normals_.resize(shading_normals_.size() + itm.vertex_indices.size());
            }

            size_t first_tex_coord = tex_coords_.size();
            if (itm.tex_coord_indices.size() > 0)
            {
                assert(itm.tex_coord_indices.size() % 3 == 0);
                tex_coords_.resize(tex_coords_.size() + itm.tex_coord_indices.size());
            }

            size_t first_color = colors_.size();
            if (itm.color_indices.size() > 0)
            {
                assert(itm.color_indices.size() % 3 == 0);
                colors_.resize(first_color + itm.color_indices.size());
            }

            for (size_t i = 0; i < itm.vertex_indices.size(); i += 3)
            {
                vec3 v1 = (*itm.vertices)[itm.vertex_indices[i]];
                vec3 v2 = (*itm.vertices)[itm.vertex_indices[i + 1]];
                vec3 v3 = (*itm.vertices)[itm.vertex_indices[i + 2]];

                basic_triangle<3, float> tri(v1, v2 - v1, v3 - v1);
                tri.prim_id = current_prim_id_++;
                tri.geom_id = current_geom_id_;
                triangles[i / 3] = tri;

                vec3 gn = normalize(cross(v2 - v1, v3 - v1));

                geometric_normals_[first_geometric_normal + i / 3] = gn;

                if (itm.normal_indices.size() == 0)
                {
                    shading_normals_[first_shading_normal + i]     = gn;
                    shading_normals_[first_shading_normal + i + 1] = gn;
                    shading_normals_[first_shading_normal + i + 2] = gn;
                }
            }

            for (size_t i = 0; i < itm.normal_indices.size(); i += 3)
            {
                shading_normals_[first_shading_normal + i]     = (*itm.normals)[itm.normal_indices[i]];
                shading_normals_[first_shading_normal + i + 1] = (*itm.normals)[itm.normal_indices[i + 1]];
                shading_normals_[first_shading_normal + i + 2] = (*itm.normals)[itm.normal_indices[i + 2]];
            }

            for (size_t i = 0; i < itm.tex_coord_indices.size(); i += 3)
            {
                tex_coords_[first_tex_coord + i]     = (*itm.tex_coords)[itm.tex_coord_indices[i]];
                tex_coords_[first_tex_coord + i + 1] = (*itm.tex_coords)[itm.tex_coord_indices[i + 1]];
                tex_coords_[first_tex_coord + i + 2] = (*itm.tex_coords)[itm.tex_coord_indices[i + 2]];
            }

            for (size_t i = 0; i < itm.color_indices.size(); i += 3)
            {
                colors_[first_color + i]     = vec3((*itm.colors)[itm.color_indices[i]]);
                colors_[first_color + i + 1] = vec3((*itm.colors)[itm.color_indices[i + 1]]);
                colors_[first_color + i + 2] = vec3((*itm.colors)[itm.color_indices[i + 2]]);
            }

#if VSNRAY_COMMON_HAVE_PTEX
            face_ids_.insert(face_ids_.end(), itm.face_ids.begin(), itm.face_ids.end());
#endif


            // Build single bvh
            if (0/*build_strategy_ == renderer::LBVH*/)
            {
                lbvh_builder builder;

                bvhs_.emplace_back(builder.build(bvh_type{}, triangles.data(), triangles.size()));
            }
            else
            {
                binned_sah_builder builder;
                builder.enable_spatial_splits(false/*build_strategy_ == renderer::Split*/);

                bvhs_.emplace_back(builder.build(bvh_type{}, triangles.data(), triangles.size()));
            }

            itm.flags() = ~(bvhs_.size() - 1);
        }

        instances_.push_back({ static_cast<int>(~itm.flags()), current_transform_ });

        node_visitor::apply(itm);
    }

    // List of surface properties to derive geom_ids from
    std::vector<std::pair<std::shared_ptr<sg::material>, std::shared_ptr<sg::texture>>> surfaces;

    // Current transform along the path
    mat4 current_transform_ = mat4::identity();


    // Storage bvhs
    aligned_vector<bvh_type>& bvhs_;

    // Instances (BVH index + transform)
    aligned_vector<Instance>& instances_;

    // Shading normals
    aligned_vector<vec3>& shading_normals_;

    // Geometric normals
    aligned_vector<vec3>& geometric_normals_;

    // Texture coordinates
    aligned_vector<vec2>& tex_coords_;

    // Vertex colors
    aligned_vector<vec3>& colors_;

#if VSNRAY_COMMON_HAVE_PTEX
    // Ptex face ids
    aligned_vector<ptex::face_id_t>& face_ids_;
#endif

    // Cameras
    aligned_vector<std::pair<std::string, thin_lens_camera>>& cameras_;

    // Point lights
    aligned_vector<point_light<float>>& point_lights_;

    // Spot lights
    aligned_vector<spot_light<float>>& spot_lights_;

    // Environment map
    visionaray::texture<vec4, 2>& env_map_;

    // Environment light
    environment_light<float, texture_ref<vec4, 2>>& env_light_;

    // Assign consecutive prim ids
    unsigned current_prim_id_ = 0;

    // Assign consecutive geom ids for each encountered material
    unsigned current_geom_id_ = 0;

};

struct SceneBuilder
{
    model& mod;
    index_bvh<bvh_type::bvh_inst>& top_level_bvh;
    aligned_vector<bvh_type>& bvhs;
    aligned_vector<bvh_type::bvh_inst>& bvh_instances;
    aligned_vector<plastic<float>>& plastic_materials;
    aligned_vector<generic_material_t>& generic_materials;
    aligned_vector<point_light<float>>& point_lights;
    aligned_vector<spot_light<float>>& spot_lights;
    aligned_vector<area_light<float, basic_triangle<3, float>>>& area_lights;
#if VSNRAY_COMMON_HAVE_PTEX
    aligned_vector<ptex::face_id_t>& ptex_tex_coords;
    aligned_vector<ptex::texture>& ptex_textures;
#endif
    std::string& env_map_filename;
    visionaray::texture<vec4, 2>& env_map;
    environment_light<float, texture_ref<vec4, 2>>& env_light;
    aligned_vector<std::pair<std::string, thin_lens_camera>>& cameras;
    TexFormat& tex_format;

    build_scene_visitor build_visitor;

    // Output instances
    aligned_vector<Instance> instances;

    SceneBuilder(
        model& mod,
        index_bvh<bvh_type::bvh_inst>& top_level_bvh,
        aligned_vector<bvh_type>& bvhs,
        aligned_vector<bvh_type::bvh_inst>& bvh_instances,
        aligned_vector<plastic<float>>& plastic_materials,
        aligned_vector<generic_material_t>& generic_materials,
        aligned_vector<point_light<float>>& point_lights,
        aligned_vector<spot_light<float>>& spot_lights,
        aligned_vector<area_light<float, basic_triangle<3, float>>>& area_lights,
#if VSNRAY_COMMON_HAVE_PTEX
        aligned_vector<ptex::face_id_t>& ptex_tex_coords,
        aligned_vector<ptex::texture>& ptex_textures,
#endif
        std::string& env_map_filename,
        visionaray::texture<vec4, 2>& env_map,
        environment_light<float, texture_ref<vec4, 2>>& env_light,
        aligned_vector<std::pair<std::string, thin_lens_camera>>& cameras,
        TexFormat& tex_format)
        : mod(mod)
        , top_level_bvh(top_level_bvh)
        , bvhs(bvhs)
        , bvh_instances(bvh_instances)
        , plastic_materials(plastic_materials)
        , generic_materials(generic_materials)
        , point_lights(point_lights)
        , spot_lights(spot_lights)
        , area_lights(area_lights)
#if VSNRAY_COMMON_HAVE_PTEX
        , ptex_tex_coords(ptex_tex_coords)
        , ptex_textures(ptex_textures)
#endif
        , env_map_filename(env_map_filename)
        , env_map(env_map)
        , env_light(env_light)
        , cameras(cameras)
        , tex_format(tex_format)
        , build_visitor(
                bvhs,
                instances,
                mod.shading_normals, // TODO!!!
                mod.geometric_normals,
                mod.tex_coords,
                mod.colors,
#if VSNRAY_COMMON_HAVE_PTEX
                ptex_tex_coords,
#endif
                cameras,
                point_lights,
                spot_lights,
                env_map,
                env_light
        )
    {
    }

    bool acquireScene()
    {
        std::cout << "Assemble scene...\n";

        if (mod.scene_graph == nullptr)
        {
            std::cerr << "Scene does not contain an instance hierarchy... quitting\n";

            return false;
        }
        else
        {
            reset_flags_visitor reset_visitor;
            mod.scene_graph->accept(reset_visitor);

            mod.scene_graph->accept(build_visitor);

            return true;
        }
    }

    void buildInstanceBVHs()
    {
        bvh_instances.resize(instances.size());
        for (size_t i = 0; i < instances.size(); ++i)
        {
            size_t index = instances[i].index;
            bvh_instances[i] = bvhs[index].inst(instances[i].transform);
        }
    }

    void buildInternalDataStructures()
    {
        // Single BVH
        if (0/*build_strategy == LBVH*/)
        {
            lbvh_builder builder;

            top_level_bvh = builder.build(
                    index_bvh<bvh_type::bvh_inst>{},
                    bvh_instances.data(),
                    bvh_instances.size()
                    );
        }
        else
        {
            binned_sah_builder builder;
            builder.enable_spatial_splits(false);

            top_level_bvh = builder.build(
                    index_bvh<bvh_type::bvh_inst>{},
                    bvh_instances.data(),
                    bvh_instances.size()
                    );
        }


        tex_format = TexFormat::UV;

#if VSNRAY_COMMON_HAVE_PTEX
        // Simply check the first texture of the first surface
        // Scene has either Ptex textures, or it doesn't
        if (build_visitor.surfaces.size() > 0
            && std::dynamic_pointer_cast<sg::ptex_texture>(build_visitor.surfaces[0].second) != nullptr)
        {
            tex_format = TexFormat::Ptex;
            ptex_textures.resize(build_visitor.surfaces.size());
        }
#endif

        // Insert dummy material (wavefront obj) if no surfaces
        // were parsed from sg
        if (build_visitor.surfaces.size() == 0)
        {
            build_visitor.surfaces.resize(1);

            // Material
            build_visitor.surfaces[0].first = std::make_shared<sg::obj_material>();

            // Texture
            vector<4, unorm<8>> dummy_texel(1.0f, 1.0f, 1.0f, 1.0f);
            auto tex = std::make_shared<sg::texture2d<vector<4, unorm<8>>>>();
            tex->resize(1, 1);
            tex->set_address_mode(Wrap);
            tex->set_filter_mode(Nearest);
            tex->reset(&dummy_texel);
            build_visitor.surfaces[0].second = tex;
        }

        mod.textures.resize(build_visitor.surfaces.size());

        for (size_t i = 0; i < build_visitor.surfaces.size(); ++i)
        {
            auto const& surf = build_visitor.surfaces[i];

            model::material_type newmat = {};

            if (auto disney = std::dynamic_pointer_cast<sg::disney_material>(surf.first))
            {
                generic_materials.emplace_back(map_material(*disney));
            }
            else if (auto obj = std::dynamic_pointer_cast<sg::obj_material>(surf.first))
            {
                generic_materials.emplace_back(map_material(*obj));
            }
            else if (auto metal = std::dynamic_pointer_cast<sg::metal_material>(surf.first))
            {
                generic_materials.emplace_back(map_material(*metal));
            }
            else if (auto glass = std::dynamic_pointer_cast<sg::glass_material>(surf.first))
            {
                generic_materials.emplace_back(map_material(*glass));
            }


#if VSNRAY_COMMON_HAVE_PTEX
            if (tex_format == TexFormat::Ptex)
            {
                auto ptex_tex = std::dynamic_pointer_cast<sg::ptex_texture>(surf.second);
                if (ptex_tex != nullptr)
                {
                    ptex_textures[i] = { ptex_tex->filename(), ptex_tex->cache() };
                }
            }
            else if (tex_format == TexFormat::UV)
#endif

            {
                auto tex = std::dynamic_pointer_cast<sg::texture2d<vector<4, unorm<8>>>>(surf.second);
                if (tex != nullptr)
                {
                    model::texture_type texture(tex->width(), tex->height());
                    texture.set_address_mode(tex->get_address_mode());
                    texture.set_filter_mode(tex->get_filter_mode());
                    texture.reset(tex->data());

                    auto it = mod.texture_map.insert(std::make_pair(tex->name(), std::move(texture)));
                    mod.textures[i] = model::texture_type::ref_type(it.first->second);
                }
            }
        }

        mod.bbox = top_level_bvh.node(0).get_bounds();
        mod.materials.push_back({});

#if 1
        mod.scene_graph.reset();
#endif

        // Generate a list with plastic materials
        plastic_materials = make_materials(
                plastic<float>{},
                mod.materials
                );

        if (generic_materials.empty())
        {
            // Generate another list with generic materials
            generic_materials = make_materials(
                    generic_material_t{},
                    mod.materials,
                    [](aligned_vector<generic_material_t>& cont, model::material_type mat)
                    {
                        cont.emplace_back(map_material(mat));
                    }
                    );
        }


        // Loop over all triangles, check if their
        // material is emissive, and if so, build
        // BVHs to create area lights from.

        struct range
        {
            std::size_t begin;
            std::size_t end;
            unsigned    bvh_id;
            unsigned    geom_id;
        };

        std::vector<range> ranges;

        for (unsigned b = 0; b < bvhs.size(); ++b)
        {
            for (std::size_t i = 0; i < bvhs[b].primitives().size(); ++i)
            {
                auto pi = bvhs[b].primitives()[i];
                if (generic_materials[pi.geom_id].as<emissive<float>>() != nullptr)
                {
                    range r;
                    r.begin = i;

                    std::size_t j = i + 1;
                    for (;j < bvhs[b].primitives().size(); ++j)
                    {
                        auto pii = bvhs[b].primitives()[j];
                        if (generic_materials[pii.geom_id].as<emissive<float>>() == nullptr
                                        || pii.geom_id != pi.geom_id)
                        {
                            break;
                        }
                    }

                    r.end = j;
                    r.bvh_id = b;
                    r.geom_id = pi.geom_id;
                    ranges.push_back(r);

                    i = r.end - 1;
                }
            }
        }

        // Build vector with area light sources
        for (auto r : ranges)
        {
            for (std::size_t i = r.begin; i != r.end; ++i)
            {
                area_light<float, basic_triangle<3, float>> light(bvhs[r.bvh_id].primitives()[i]);
                auto mat = *generic_materials[r.geom_id].as<emissive<float>>();
                light.set_cl(to_rgb(mat.ce()));
                light.set_kl(mat.ls());
                area_lights.push_back(light);
            }
        }
    }

};

} // visionaray
