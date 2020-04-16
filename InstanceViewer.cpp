// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <exception>
#include <iostream>
#include <ostream>
#include <set>
#include <string>
#include <vector>
#include <unordered_map>
#include <vector>

#include <GL/glew.h>

#include <boost/filesystem.hpp>

#include <visionaray/math/math.h>
#define private public //:-)
#include <visionaray/bvh.h>
#undef private
#include <visionaray/cpu_buffer_rt.h>
#include <visionaray/kernels.h>
#include <visionaray/scheduler.h>
#include <visionaray/thin_lens_camera.h>

#include <Support/CmdLine.h>
#include <Support/CmdLineUtil.h>

// Private visionaray_common includes!
#include <common/config.h>

#include <common/manip/arcball_manipulator.h>
#include <common/manip/pan_manipulator.h>
#include <common/manip/zoom_manipulator.h>

#include <common/model.h>
#include <common/png_image.h>
#include <common/sg.h>
#include <common/timer.h>

#if VSNRAY_COMMON_HAVE_PTEX
#include <common/ptex.h>
#endif

#if VSNRAY_COMMON_HAVE_SDL2
#include <common/viewer_sdl2.h>
#else
#include <common/viewer_glut.h>
#endif

#include "BuildScene.hpp"
#include "kd_tree.h"
#include "Pathtracer.hpp"
#include "svt.h"

using namespace visionaray;

#if VSNRAY_COMMON_HAVE_SDL2
using ViewerBase = viewer_sdl2;
#else
using ViewerBase = viewer_glut;
#endif

//-------------------------------------------------------------------------------------------------
// I/O utility for camera lookat only - not fit for the general case!
//

std::istream& operator>>(std::istream& in, pinhole_camera& cam)
{
    vec3 eye;
    vec3 center;
    vec3 up;

    in >> eye >> std::ws >> center >> std::ws >> up >> std::ws;
    cam.look_at(eye, center, up);

    return in;
}

std::ostream& operator<<(std::ostream& out, pinhole_camera const& cam)
{
    out << cam.eye() << '\n';
    out << cam.center() << '\n';
    out << cam.up() << '\n';
    return out;
}


struct InstanceViewer : ViewerBase
{
    InstanceViewer()
        : ViewerBase(800, 800, "InstanceViewer")
        , sched(std::thread::hardware_concurrency())
    {
        using namespace support;

        // Add cmdline options
        add_cmdline_option( cl::makeOption<std::set<std::string>&>(
            cl::Parser<>(),
            "filenames",
            cl::Desc("Input file(s)"),
            cl::Positional,
            cl::OneOrMore,
            cl::init(filenames)
            ) );

        add_cmdline_option( cl::makeOption<unsigned&>(
            cl::Parser<>(),
            "n",
            cl::Desc("num nodes"),
            cl::Optional,
            cl::init(n)
            ) );

        add_cmdline_option( cl::makeOption<float&>(
            cl::Parser<>(),
            "w1",
            cl::Desc("min-rep"),
            cl::Optional,
            cl::init(w1)
            ) );

        add_cmdline_option( cl::makeOption<float&>(
            cl::Parser<>(),
            "w2",
            cl::Desc("equal-geo"),
            cl::Optional,
            cl::init(w2)
            ) );

        add_cmdline_option( cl::makeOption<float&>(
            cl::Parser<>(),
            "w3",
            cl::Desc("median"),
            cl::Optional,
            cl::init(w3)
            ) );

        add_cmdline_option( cl::makeOption<float&>(
            cl::Parser<>(),
            "w4",
            cl::Desc("middle"),
            cl::Optional,
            cl::init(w4)
            ) );

        add_cmdline_option( cl::makeOption<bool&>(
            cl::Parser<>(),
            "h",
            cl::Desc("Run w/o display, generate stats, and exit immediately"),
            cl::Optional,
            cl::init(headless)
            ) );
    }

    void clear_frame();
    void on_display();
    void on_key_press(key_event const& event);
    void on_mouse_move(visionaray::mouse_event const& event);
    void on_space_mouse_move(visionaray::space_mouse_event const& event);
    void on_resize(int w, int h);

    // Scene data
    std::set<std::string> filenames;
    model mod;
    //thin_lens_camera cam;
    pinhole_camera cam;

    // The top level BVH over instances
    index_bvh<bvh_type::bvh_inst> top_level_bvh;

    // The "storage" BVHs associated with the BVH instances
    aligned_vector<bvh_type> bvhs;

    // Special treatment for BVHs that are referenced only once
    // These are set up after the other BVHs and we store them
    // in a separate vector to avoid the std::vector copy-construct litany
    aligned_vector<bvh_type> baseMeshBvhs;

    // The BVH instances
    aligned_vector<bvh_type::bvh_inst> bvh_instances;

    // K-d tree leaf nodes
    aligned_vector<kd_tree_node> leaves;

    aligned_vector<plastic<float>> plastic_materials;
    aligned_vector<generic_material_t> generic_materials;
    aligned_vector<point_light<float>> point_lights;
    aligned_vector<spot_light<float>> spot_lights;
    aligned_vector<area_light<float, basic_triangle<3, float>>> area_lights;
#if VSNRAY_COMMON_HAVE_PTEX
    aligned_vector<ptex::face_id_t> ptex_tex_coords;
    aligned_vector<ptex::texture> ptex_textures;
#endif
    std::string env_map_filename;
    visionaray::texture<vec4, 2> env_map;
    environment_light<float, texture_ref<vec4, 2>> env_light;
    // List of cameras, e.g. read from scene graph
    aligned_vector<std::pair<std::string, thin_lens_camera>> cameras;
    TexFormat tex_format;

    // Stuff for rendering
    cpu_buffer_rt<PF_RGBA32F, PF_UNSPECIFIED> rt;
    tiled_sched<basic_ray<float>> sched;
    unsigned frame_num;

    std::vector<unsigned> geom_id_list;

    unsigned n = 4;
    float w1 = 1.f;
    float w2 = 1.f;
    float w3 = 1.f;
    float w4 = 1.f;
    bool headless = false;

    void load_camera(std::string filename)
    {
        std::ifstream file(filename);
        if (file.good())
        {
            file >> cam;
            clear_frame();
            std::cout << "Load camera from file: " << filename << '\n';
        }
    }

    void computeRankBounds(std::vector<aabb>& rankBounds,
                           std::vector<unsigned> const& primitives_to_leaves,
                           unsigned numRanks)
    {
        rankBounds.resize(numRanks);
        for (size_t i = 0; i < numRanks; ++i)
            rankBounds[i].invalidate();

        for (size_t i = 0; i < primitives_to_leaves.size(); ++i)
        {
            aabb bbox = bvh_instances[i].node(0).get_bounds();
            mat4 trans = inverse(bvh_instances[i].transform_inv());
            auto verts = compute_vertices(bbox);
            for (auto& v : verts)
                v = (trans * vec4(v, 1.f)).xyz();

            for (unsigned n = 0; n < numRanks; ++n) {
                unsigned leaf_bit = 1 << n;
                if (primitives_to_leaves[i] & leaf_bit) {
                    for (auto v : verts)
                        rankBounds[n].insert(v);
                }
            }
        }
    }

    void splitBaseMeshInstances(size_t num,
                                aligned_vector<kd_tree_node> const& leaves,
                                std::vector<unsigned>& primitives_to_leaves)
    {
        // Conservatively resize the base mesh vector in
        // advance so we later won't have to copy BVHs
        baseMeshBvhs.resize(num * leaves.size());
        size_t baseMeshIndex = 0;

        // We assume that the instance vector is sorted by
        // instance count and that the first num instances
        // are base mesh references that need to be split
        for (size_t i = 0; i < num; ++i)
        {
            // TODO: maybe take from sceneBuilder.instances?
            mat4 transform = inverse(bvh_instances[i].transform_inv());

            for (size_t j = 0; j < leaves.size(); ++j)
            {
                if (leaves[j].bbox.invalid())
                    continue;

                aligned_vector<basic_triangle<3, float>> primitives;

                for (unsigned k = 0; k < bvh_instances[i].num_indices(); ++k)
                {
                    auto tri = bvh_instances[i].primitive(k);
                    vec4 v1(tri.v1, 1.f);
                    vec4 v2(tri.v1 + tri.e1, 1.f);
                    vec4 v3(tri.v1 + tri.e2, 1.f);

                    v1 = transform * v1;
                    v2 = transform * v2;
                    v3 = transform * v3;

                    if (leaves[j].bbox.contains(v1.xyz())
                     || leaves[j].bbox.contains(v2.xyz())
                     || leaves[j].bbox.contains(v3.xyz()))
                    {
                        primitives.push_back(tri);
                    }
                }

                if (!primitives.empty())
                {
                    if (0/*build_strategy == LBVH*/)
                    {
                        lbvh_builder builder;

                        baseMeshBvhs[baseMeshIndex] = builder.build(
                                index_bvh<basic_triangle<3, float>>{},
                                primitives.data(),
                                primitives.size()
                                );
                    }
                    else
                    {
                        binned_sah_builder builder;
                        builder.enable_spatial_splits(false);

                        baseMeshBvhs[baseMeshIndex] = builder.build(
                                index_bvh<basic_triangle<3, float>>{},
                                primitives.data(),
                                primitives.size()
                                );
                    }

                    // Add a single instances referencing the split BVH
                    bvh_instances.push_back(
                        baseMeshBvhs[baseMeshIndex].inst(transform)
                        );

                    // Also store the mapping from instance to leaf index
                    primitives_to_leaves.push_back(leaves[j].leaf_bit);


                    ++baseMeshIndex;
                }
            }
        }

        // Finally delete the no longer needed instances
        bvh_instances.erase(bvh_instances.begin(), bvh_instances.begin() + num);
    }

inline vec3f randomColor(size_t idx)
    {
      unsigned int r = (unsigned int)(idx*13*17 + 0x234235);
      unsigned int g = (unsigned int)(idx*7*3*5 + 0x773477);
      unsigned int b = (unsigned int)(idx*11*19 + 0x223766);
      return vec3f((r&255)/255.f,
                   (g&255)/255.f,
                   (b&255)/255.f);
    }

    void assignColors(std::vector<unsigned>& primitives_to_leaves, unsigned numRanks)
    {
        mod.materials.resize(numRanks);
        for (size_t i=0; i<numRanks; ++i) {
            mod.materials[i].cd = randomColor(i);
            mod.materials[i].cs = vec3(0.0f, 0.0f, 0.0f);
        }


        geom_id_list.resize(primitives_to_leaves.size());
        for (size_t i = 0; i < primitives_to_leaves.size(); ++i)
        {
            for (unsigned n = 0; n < numRanks; ++n) {
                unsigned leaf_bit = 1 << n;
                if (primitives_to_leaves[i] & leaf_bit) {
                        geom_id_list[i] = n;
                }
            }
            //for (unsigned n = 0; n < numRanks; ++n)
            //{
            //    unsigned leaf_bit = 1 << n;
            //    if (primitives_to_leaves[i] & leaf_bit) {
            //        if (geom_id_list[i] == 0)
            //            geom_id_list[i] = n+1;
            //        else
            //            geom_id_list[i] = RedId;
            //    }
            //}

//#if 1 // Hack to make isMountainA base mesh gray
//            if (i >= primitives_to_leaves.size()-4)
//                geom_id_list[i] = GrayId;
//#endif
//
//            if (geom_id_list[i]==0)
//                std::cout << primitives_to_leaves[i] << '\n';
        }
    }

    void dump(std::vector<unsigned>& primitives_to_leaves, unsigned numRanks)
    {
        struct Triangle
        {
            float v1[3];
            float v2[3];
            float v3[3];
        };

        struct BoundingBox
        {
            float min[3];
            float max[3];
        };

        struct Instance
        {
            size_t baseMeshID;
            float transform[16]; // col0;col1;col2;col3
        };

        typedef std::vector<Triangle> BaseMesh;
        typedef std::vector<BaseMesh> BaseMeshes;
        typedef std::vector<Instance> Instances;

        BaseMeshes baseMeshes;
        std::vector<Instances> instances(numRanks);

        std::unordered_map<bvh_type::bvh_ref::P*, size_t> refmap;
        std::vector<std::pair<bvh_type::bvh_ref::P*,bvh_type::bvh_ref::P*>> refmap_inv;

        // Serialize instances
        for (size_t i = 0; i < primitives_to_leaves.size(); ++i)
        {
            auto ref = bvh_instances[i].get_ref();

            // Find out base mesh ID, potentially add to
            // base map list
            size_t baseMeshID(-1);
            auto it = refmap.find(ref.primitives_first);
            if (it == refmap.end())
            {
                baseMeshID = refmap.size();
                refmap.insert({ref.primitives_first, baseMeshID});
                refmap_inv.push_back({ref.primitives_first, ref.primitives_last});

                assert(refmap.size() == refmap_inv.size());

            }
            else
                baseMeshID = it->second;

            mat4 transform = inverse(bvh_instances[i].transform_inv());
            Instance inst;
            inst.baseMeshID = baseMeshID;
            inst.transform[ 0] = transform.col0.x;
            inst.transform[ 1] = transform.col0.y;
            inst.transform[ 2] = transform.col0.z;
            inst.transform[ 3] = transform.col0.w;
            inst.transform[ 4] = transform.col1.x;
            inst.transform[ 5] = transform.col1.y;
            inst.transform[ 6] = transform.col1.z;
            inst.transform[ 7] = transform.col1.w;
            inst.transform[ 8] = transform.col2.x;
            inst.transform[ 9] = transform.col2.y;
            inst.transform[10] = transform.col2.z;
            inst.transform[11] = transform.col2.w;
            inst.transform[12] = transform.col3.x;
            inst.transform[13] = transform.col3.y;
            inst.transform[14] = transform.col3.z;
            inst.transform[15] = transform.col3.w;

            for (unsigned n = 0; n < numRanks; ++n)
            {
                unsigned leaf_bit = 1 << n;
                if (primitives_to_leaves[i] & leaf_bit)
                {
                    instances[n].push_back(inst);
                }
            }
        }

        // Serialize base meshes
        baseMeshes.resize(refmap_inv.size());
        for (size_t i = 0; i < refmap_inv.size(); ++i)
        {
            auto first = refmap_inv[i].first;
            auto last  = refmap_inv[i].second;
            for (auto P = first; P != last; ++P)
            {
                auto tri = *P;
                vec3 v1 = tri.v1;
                vec3 v2 = tri.e1 + tri.v1;
                vec3 v3 = tri.e2 + tri.v1;
                // Serialize out 16 byte alignment of vec3:
                Triangle triSer = {
                    { v1.x, v1.y, v1.z }, { v2.x, v2.y, v2.z }, { v3.x, v3.y, v3.z } 
                    };
                baseMeshes[i].push_back(triSer);
            }
        }

        // Compute object bounds
        std::vector<aabb> rankBounds;
        computeRankBounds(rankBounds, primitives_to_leaves, numRanks);

        std::vector<BoundingBox> objBounds(numRanks);
        std::vector<BoundingBox> domainBounds(numRanks);

        for (size_t i = 0; i < numRanks; ++i)
        {
            // Serialize out 16 byte alignment of vec3:
            aabb obj = rankBounds[i];
            aabb dom = leaves[i].bbox;
            objBounds[i] = { {obj.min.x, obj.min.y, obj.min.z}, {obj.max.x, obj.max.y, obj.max.z} };
            domainBounds[i] = { {dom.min.x, dom.min.y, dom.min.z}, {dom.max.x, dom.max.y, dom.max.z} };
        }
        for (int i=0; i<numRanks; ++i)
        {
            for (int j=0; j<numRanks; ++j)
            {
                if (i==j)
                    continue;
                aabb b = intersect(leaves[i].bbox,leaves[j].bbox);
                if (b.valid() && volume(b) > 0.f) {
                    std::cout << i << ' ' << leaves[i].bbox.min << leaves[i].bbox.max << '\n';
                    std::cout << j << ' ' << leaves[j].bbox.min << leaves[j].bbox.max << '\n';
                    std::cout << b.min << b.max << '\n';
                    std::cout << "Overlap volume: " << volume(b) << '\n';
                }
            }
        }

        boost::filesystem::path p(*filenames.begin());
        std::string base = p.stem().string();
        std::stringstream str;
        str << base << "_n" << n << '_' << (int)w1 << (int)w2 << (int)w3 << (int)w4 << ".bin";
        std::string fn = str.str();

        std::cout << "Writing out to " << fn << '\n';

        // Write file
        std::ofstream file(fn);                                                                     // BEGIN
        //std::cout << "Writing " << baseMeshes.size() << " base meshes\n";
        uint64_t numBaseMeshes = baseMeshes.size();
        file.write((char*)&numBaseMeshes, sizeof(numBaseMeshes));                                   // 64-bit num base meshes
        for (size_t i = 0; i < baseMeshes.size(); ++i)
        {
            uint64_t numTriangles = baseMeshes[i].size();
            file.write((char*)&numTriangles, sizeof(numTriangles));                                 // 64-bit num triangles
            file.write((char*)baseMeshes[i].data(), baseMeshes[i].size() * sizeof(Triangle));       // triangle list
        }

        uint64_t numRanks64(numRanks);
        file.write((char*)&numRanks64, sizeof(numRanks64));                                         // 64-bit num ranks

        for (unsigned n = 0; n < numRanks; ++n)
        {
            std::cout << "Writing object bounds " << n << '\n';
            BoundingBox obj = objBounds[n];
            file.write((char*)&obj, sizeof(obj));                                                   // 6x float object bounds

            std::cout << "Writing domain bounds " << n << '\n';
            BoundingBox dom = domainBounds[n];
            file.write((char*)&dom, sizeof(dom));                                                   // 6x float domain bounds

            std::cout << "Writing " << instances[n].size() << " instances for rank " << n << '\n';
            uint64_t numInstances = instances[n].size();
            file.write((char*)&numInstances, sizeof(numInstances));                                 // 64-bit num instances
            file.write((char*)instances[n].data(), instances[n].size() * sizeof(Instance));         // instance list
        }                                                                                           // END
    }
};


void InstanceViewer::clear_frame()
{
    frame_num = 0;
    rt.clear_color_buffer();
}

void InstanceViewer::on_display()
{
    pixel_sampler::jittered_blend_type blend_params;
    float alpha = 1.0f / ++frame_num;
    blend_params.sfactor = alpha;
    blend_params.dfactor = 1.0f - alpha;

    using bvh_ref = index_bvh<index_bvh<basic_triangle<3, float>>::bvh_inst>::bvh_ref;

    aligned_vector<bvh_ref> primitives;

    primitives.push_back(top_level_bvh.ref());

    aligned_vector<generic_light_t> temp_lights;
    for (auto pl : point_lights)
    {
        temp_lights.push_back(pl);
    }

    for (auto sl : spot_lights)
    {
        temp_lights.push_back(sl);
    }

    for (auto al : area_lights)
    {
        temp_lights.push_back(al);
    }

    auto bounds = mod.bbox;
    auto diagonal = bounds.max - bounds.min;
    auto epsilon = std::max( 1E-3f, length(diagonal) * 1E-5f );

    auto sparams = make_sched_params(
            blend_params,
            cam,
            rt
            );

    //if (tex_format == TexFormat::UV)
    {
        auto kparams = make_kernel_params(
                normals_per_vertex_binding{},
                primitives.data(),
                primitives.data() + primitives.size(),
                mod.geometric_normals.data(),
                mod.shading_normals.data(),
                //mod.tex_coords.data(),
                //generic_materials.data(),
                plastic_materials.data(),
                //mod.textures.data(),
                temp_lights.data(),
                temp_lights.data() + temp_lights.size(),
                10,
                epsilon,
                vec4(background_color(), 1.0f),
                vec4(1,1,1,1)
                );

        Pathtracer<decltype(kparams)> kernel(geom_id_list);
        kernel.params = kparams;
        sched.frame(kernel, sparams);
    }
    /*else
    {
        auto kparams = make_kernel_params(
                normals_per_vertex_binding{},
                primitives.data(),
                primitives.data() + primitives.size(),
                mod.geometric_normals.data(),
                mod.shading_normals.data(),
                ptex_tex_coords.data(),
                generic_materials.data(),
                ptex_textures.data(),
                temp_lights.data(),
                temp_lights.data() + temp_lights.size(),
                10,
                epsilon,
                vec4(background_color(), 1.0f),
                vec4(1,1,1,1)
                );

        pathtracing::kernel<decltype(kparams)> kernel;
        kernel.params = kparams;
        sched.frame(kernel, sparams);
    }*/

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //glEnable(GL_FRAMEBUFFER_SRGB);
    rt.display_color_buffer();

    if (false)
    {
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadMatrixf(cam.get_proj_matrix().data());

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadMatrixf(cam.get_view_matrix().data());

        glBegin(GL_LINES);
        glLineWidth(3.f);
        glColor3f(1.f,1.f,1.f);

        glEnable(GL_DEPTH_CLAMP);

        for (size_t i = 0; i < leaves.size(); ++i)
        {
            auto verts = compute_vertices(leaves[i].bbox);
            glVertex3f(verts[0].x,verts[0].y,verts[0].z);
            glVertex3f(verts[1].x,verts[1].y,verts[1].z);

            glVertex3f(verts[1].x,verts[1].y,verts[1].z);
            glVertex3f(verts[2].x,verts[2].y,verts[2].z);

            glVertex3f(verts[2].x,verts[2].y,verts[2].z);
            glVertex3f(verts[3].x,verts[3].y,verts[3].z);

            glVertex3f(verts[3].x,verts[3].y,verts[3].z);
            glVertex3f(verts[0].x,verts[0].y,verts[0].z);


            glVertex3f(verts[4].x,verts[4].y,verts[4].z);
            glVertex3f(verts[5].x,verts[5].y,verts[5].z);

            glVertex3f(verts[5].x,verts[5].y,verts[5].z);
            glVertex3f(verts[6].x,verts[6].y,verts[6].z);

            glVertex3f(verts[6].x,verts[6].y,verts[6].z);
            glVertex3f(verts[7].x,verts[7].y,verts[7].z);

            glVertex3f(verts[7].x,verts[7].y,verts[7].z);
            glVertex3f(verts[4].x,verts[4].y,verts[4].z);

            glVertex3f(verts[0].x,verts[0].y,verts[0].z);
            glVertex3f(verts[5].x,verts[5].y,verts[5].z);

            glVertex3f(verts[1].x,verts[1].y,verts[1].z);
            glVertex3f(verts[4].x,verts[4].y,verts[4].z);

            glVertex3f(verts[2].x,verts[2].y,verts[2].z);
            glVertex3f(verts[7].x,verts[7].y,verts[7].z);

            glVertex3f(verts[3].x,verts[3].y,verts[3].z);
            glVertex3f(verts[6].x,verts[6].y,verts[6].z);
        }
        glEnd();

        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
    }
}

void InstanceViewer::on_key_press(key_event const& event)
{
    static const std::string camera_file_base = "viewer-camera";
    static const std::string camera_file_suffix = ".txt";

    static const std::string screenshot_filename = "screenshot.pnm";

    switch (event.key())
    {

    case 'p':
        {
            // Swizzle to RGB8 for compatibility with pnm image
            std::vector<vector<3, unorm<8>>> rgb(rt.width() * rt.height());
            swizzle(
                rgb.data(),
                PF_RGB8,
                rt.color(),
                PF_RGBA32F,
                rt.width() * rt.height(),
                TruncateAlpha
                );

            if (1)//rt.color_space() == host_device_rt::SRGB)
            {
                for (int y = 0; y < rt.height(); ++y)
                {
                    for (int x = 0; x < rt.width(); ++x)
                    {
                        auto& color = rgb[y * rt.width() + x];
                        color.x = std::pow(color.x, 1 / 2.2f);
                        color.y = std::pow(color.y, 1 / 2.2f);
                        color.z = std::pow(color.z, 1 / 2.2f);
                    }
                }
            }

            image img(
                rt.width(),
                rt.height(),
                PF_RGB8,
                reinterpret_cast<uint8_t const*>(rgb.data())
                );

            image::save_option opt1({"binary", true});
            if (img.save(screenshot_filename, {opt1}))
            {
                std::cout << "Screenshot saved to file: " << screenshot_filename << '\n';
            }
            else
            {
                std::cerr << "Error saving screenshot to file: " << screenshot_filename << '\n';
            }

        }
        break;

    case 'u':
        {
            int inc = 0;
            std::string inc_str = "";

            std::string filename = camera_file_base + inc_str + camera_file_suffix;

            while (boost::filesystem::exists(filename))
            {
                ++inc;
                inc_str = std::to_string(inc);

                while (inc_str.length() < 4)
                {
                    inc_str = std::string("0") + inc_str;
                }

                inc_str = std::string("-") + inc_str;

                filename = camera_file_base + inc_str + camera_file_suffix;
            }

            std::ofstream file(filename);
            if (file.good())
            {
                std::cout << "Storing camera to file: " << filename << '\n';
                file << cam;
            }
        }
        break;

    case 'v':
        {
            std::string filename = camera_file_base + camera_file_suffix;

            load_camera(filename);
        }
        break;

    default:
        break;
    }

    ViewerBase::on_key_press(event);
}

void InstanceViewer::on_mouse_move(visionaray::mouse_event const& event)
{
    if (event.buttons() != mouse::NoButton)
    {
        clear_frame();
    }

    ViewerBase::on_mouse_move(event);
}

void InstanceViewer::on_space_mouse_move(visionaray::space_mouse_event const& event)
{
    clear_frame();

    ViewerBase::on_space_mouse_move(event);
}

void InstanceViewer::on_resize(int w, int h)
{
    cam.set_viewport(0, 0, w, h);
    float fovy = cam.fovy();
    float aspect = w / static_cast<float>(h);
    float z_near = cam.z_near();
    float z_far = cam.z_far();
    cam.perspective(fovy, aspect, z_near, z_far);
    rt.resize(w, h);
    clear_frame();
    ViewerBase::on_resize(w, h);
}


int main(int argc, char** argv)
{
    InstanceViewer viewer;

    try
    {
        viewer.init(argc, argv);
    }
    catch (std::exception const& e)
    {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

	if (viewer.filenames.empty())
	{
		std::cout << viewer.cmd_line_inst().help(argv[0]) << "\n";
		return EXIT_FAILURE;
	}

    std::vector<std::string> filenames;
    std::copy(viewer.filenames.begin(), viewer.filenames.end(), std::back_inserter(filenames));

    if (!viewer.mod.load(filenames))
    {
        std::cerr << "Failed loading model\n";
        return EXIT_FAILURE;
    }

    SceneBuilder sceneBuilder(viewer.mod,
                              viewer.top_level_bvh,
                              viewer.bvhs,
                              viewer.bvh_instances,
                              viewer.plastic_materials,
                              viewer.generic_materials,
                              viewer.point_lights,
                              viewer.spot_lights,
                              viewer.area_lights,
#if VSNRAY_COMMON_HAVE_PTEX
                              viewer.ptex_tex_coords,
                              viewer.ptex_textures,
#endif
                              viewer.env_map_filename,
                              viewer.env_map,
                              viewer.env_light,
                              viewer.cameras,
                              viewer.tex_format);

    if (!sceneBuilder.acquireScene())
    {
        return EXIT_FAILURE;
    }

    // Sort instances by instance count
    std::vector<int> inst_counts(sceneBuilder.instances.size());
    std::fill(inst_counts.begin(), inst_counts.end(), 0);

    for (size_t i = 0; i < sceneBuilder.instances.size(); ++i)
    {
        size_t index = sceneBuilder.instances[i].index;

        inst_counts[index]++;
    }

    std::sort(sceneBuilder.instances.begin(), sceneBuilder.instances.end(),
            [&inst_counts](Instance a, Instance b)
            {
                return inst_counts[a.index] < inst_counts[b.index];
            });

    sceneBuilder.buildInstanceBVHs();

    // Find the first instance that is instanced more than once
    size_t first_index(-1);
    for (size_t i = 0; i < sceneBuilder.instances.size(); ++i)
    {
        if (inst_counts[sceneBuilder.instances[i].index] > 1)
        {
            first_index = i;//std::cout << "Found " << i << '\n';
            break;
        }
    }
    if (first_index == size_t(-1))
        first_index = 0;

    timer t;

    // Build an SVT for the geometry that is instanced only once
    SVT<unsigned> svt;
    svt.init(viewer.bvh_instances.data(), first_index, vec3i(256, 256, 256));

    std::cout << "SVT: " << t.elapsed() << '\n';
    t.reset();

    int numRanks = viewer.n;

    float w1 = viewer.w1;
    float w2 = viewer.w2;
    float w3 = viewer.w3;
    float w4 = viewer.w4;

    std::cout << "w1: " << w1 << ", w2: " << w2 << ", w3: " << w3 << ", w4: " << w4 << '\n';
    kd_tree<bvh_type::bvh_inst> kdtree(svt, w1,w2,w3,w4);
    kdtree.init(viewer.bvh_instances.data() + first_index,
                viewer.bvh_instances.size() - first_index,
                sceneBuilder.instances.data(),
                viewer.bvhs.size(),
                numRanks);

    std::cout << "Build: " << t.elapsed() << '\n';
    t.reset();

    // Split the (formerly virtual) base mesh instances
    viewer.splitBaseMeshInstances(first_index, kdtree.leaves_, kdtree.primitives_to_leaves_);

    std::cout << "Split: " << t.elapsed() << '\n';
    t.reset();

    // Assign debug colors
    viewer.assignColors(kdtree.primitives_to_leaves_, numRanks);

    // Store these so we can later superimpose their outlines
    viewer.leaves = kdtree.leaves_;

    // Write out to format that Ingo can import
    //viewer.dump(kdtree.primitives_to_leaves_, numRanks);

    if (viewer.headless)
        return EXIT_SUCCESS;

    // Build top level BVH
    sceneBuilder.buildInternalDataStructures();

    float aspect = viewer.width() / static_cast<float>(viewer.height());

    viewer.cam.perspective(45.0f * constants::degrees_to_radians<float>(), aspect, 0.001f, 1000.0f);
    //viewer.cam.set_lens_radius(0.02f);
    //viewer.cam.set_focal_distance(10.0f);

    viewer.cam.view_all(viewer.mod.bbox);

    viewer.add_manipulator(std::make_shared<arcball_manipulator>(viewer.cam, mouse::Left));
    viewer.add_manipulator(std::make_shared<pan_manipulator>(viewer.cam, mouse::Middle));
    // Additional "Alt + LMB" pan manipulator for setups w/o middle mouse button
    viewer.add_manipulator(std::make_shared<pan_manipulator>(viewer.cam, mouse::Left, keyboard::Alt));
    viewer.add_manipulator(std::make_shared<zoom_manipulator>(viewer.cam, mouse::Right));

    viewer.event_loop();
}
