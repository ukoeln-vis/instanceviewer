// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <map>

#include <visionaray/get_area.h>
#include <visionaray/get_surface.h>
#include <visionaray/result_record.h>
#include <visionaray/sampling.h>
#include <visionaray/spectrum.h>
#include <visionaray/surface.h>
#include <visionaray/surface_interaction.h>
#include <visionaray/traverse.h>

namespace visionaray
{

inline 
    vec3f missColor(int y)
    {
      const float t = y / 800.f;
      const vec3f c = (1.0f - t)*vec3f(1.0f, 1.0f, 1.0f) + t * vec3f(0.5f, 0.7f, 1.0f);
      return c;
    }


template <typename Light, typename R, typename T = typename R::scalar_type>
VSNRAY_FUNC
vector<4, T> sample_environment_light(Light const& env_light, R ray)
{
    auto dir = (matrix<4, 4, T>(env_light.world_to_light_transform()) * vector<4, T>(ray.dir, T(0.0))).xyz();

    auto x = atan2(dir.x, dir.z);
    x = select(x < T(0.0), x + constants::two_pi<T>(), x);
    auto y = acos(dir.y);

    auto u = x / constants::two_pi<T>();
    auto v = y * constants::inv_pi<T>();

    vector<2, T> tc(u, v);

    return tex2D(env_light.texture(), tc) * vector<4, T>(to_rgba(env_light.scale()));
}

template <typename R, typename T = typename R::scalar_type>
VSNRAY_FUNC
vector<4, T> sample_environment_light(std::nullptr_t*, R)
{
    return vector<4, T>(0.0);
}

template <typename Params>
struct Pathtracer
{
    Pathtracer(std::vector<unsigned> const& gil) : geom_id_list(gil) {}

    Params params;

    template <typename Intersector, typename R, typename Generator>
    VSNRAY_FUNC result_record<typename R::scalar_type> operator()(
            Intersector& isect,
            R ray,
            Generator& gen,
            int x,
            int y
            ) const
    {
        using S = typename R::scalar_type;
        using I = simd::int_type_t<S>;
        using V = typename result_record<S>::vec_type;
        using C = spectrum<S>;

        simd::mask_type_t<S> active_rays = true;
        simd::mask_type_t<S> last_specular = true;

        C intensity(0.0);
        C throughput(1.0);

        result_record<S> result;
        result.color = params.bg_color;

        //if (params.environment_map)
        {
            vec4f miss(missColor(y),1.f);
            result.color = miss;//sample_environment_light(params.environment_map, ray);
        }

        for (unsigned bounce = 0; bounce < params.num_bounces; ++bounce)
        {
            auto hit_rec = closest_hit(ray, params.prims.begin, params.prims.end, isect);

            // Handle rays that just exited
            auto exited = active_rays & !hit_rec.hit;

            if (params.environment_map)
            {
                auto env = sample_environment_light(params.environment_map, ray);
                intensity += select(
                    exited,
                    from_rgba(env) * throughput,
                    C(0.0)
                    );
            }
            else
            {
                vec4f miss(missColor(y),1.f);
                intensity += select(
                    exited,
                    C(from_rgba(miss)) * throughput,
                    C(0.0)
                    );
            }


            // Exit if no ray is active anymore
            active_rays &= hit_rec.hit;

            if (!any(active_rays))
            {
                break;
            }

            // Special handling for first bounce
            if (bounce == 0)
            {
                result.hit = hit_rec.hit;
                result.isect_pos = ray.ori + ray.dir * hit_rec.t;
            }


            // Process the current bounce

            V refl_dir(0.0);
            V view_dir = -ray.dir;

            hit_rec.isect_pos = ray.ori + ray.dir * hit_rec.t;

            if (instance_color_mode)
            {
                // This is super hacky, but the most straightforward way
                // to get the geom id based on the instance object
                hit_rec.geom_id = geom_id_list[params.prims.begin[0].indices_first[hit_rec.primitive_list_index]];
                //hit_rec.geom_id = geom_id_list[hit_rec.primitive_list_index];
            }

            auto surf = get_surface(hit_rec, params);

            S brdf_pdf(0.0);

            // Remember the last type of surface interaction.
            // If the last interaction was not diffuse, we have
            // to include light from emissive surfaces.
            I inter = 0;
            auto src = surf.sample(view_dir, refl_dir, brdf_pdf, inter, gen);

            auto zero_pdf = brdf_pdf <= S(0.0);

            S light_pdf(0.0);
            auto num_lights = params.lights.end - params.lights.begin;

            if (num_lights > 0 && any(inter == surface_interaction::Emission))
            {
                auto A = get_area(params.prims.begin, hit_rec);
                auto ld = length(hit_rec.isect_pos - ray.ori);
                auto L = normalize(hit_rec.isect_pos - ray.ori);
                auto n = surf.geometric_normal;
                auto ldotln = abs(dot(-L, n));
                auto solid_angle = (ldotln * A) / (ld * ld);

                light_pdf = select(
                    inter == surface_interaction::Emission,
                    S(1.0) / solid_angle,
                    S(0.0)
                    );
            }

            S mis_weight = select(
                bounce > 0 && num_lights > 0 && !last_specular,
                power_heuristic(brdf_pdf, light_pdf / static_cast<float>(num_lights)),
                S(1.0)
                );

            intensity += select(
                active_rays && inter == surface_interaction::Emission,
                mis_weight * throughput * src,
                C(0.0)
                );

            active_rays &= inter != surface_interaction::Emission;
            active_rays &= !zero_pdf;

            auto n = surf.shading_normal;
#if 1
            n = faceforward( n, view_dir, surf.geometric_normal );
#endif

            if (num_lights > 0)
            {
                auto ls = sample_random_light(params.lights.begin, params.lights.end, gen);

                auto ld = length(ls.pos - hit_rec.isect_pos);
                auto L = normalize(ls.pos - hit_rec.isect_pos);

                auto ln = select(ls.delta_light, -L, ls.normal);
#if 1
                ln = faceforward( ln, -L, ln );
#endif
                auto ldotn = dot(L, n);
                auto ldotln = abs(dot(-L, ln));

                R shadow_ray(
                    hit_rec.isect_pos + L * S(params.epsilon),
                    L
                    );

                auto lhr = any_hit(shadow_ray, params.prims.begin, params.prims.end, ld - S(2.0f * params.epsilon), isect);

                auto brdf_pdf = surf.pdf(view_dir, L, inter);
                auto prob = max_element(throughput.samples());
                brdf_pdf *= prob;

                // TODO: inv_pi / dot(n, wi) factor only valid for plastic and matte
                auto src = surf.shade(view_dir, L, ls.intensity) * constants::inv_pi<S>() / ldotn;
                auto solid_angle = (ldotln * ls.area);
                solid_angle = select(!ls.delta_light, solid_angle / (ld * ld), solid_angle);
                auto light_pdf = S(1.0) / solid_angle;

                S mis_weight = power_heuristic(light_pdf / static_cast<float>(num_lights), brdf_pdf);

                intensity += select(
                    active_rays && !lhr.hit && ldotn > S(0.0) && ldotln > S(0.0),
                    mis_weight * throughput * src * (ldotn / light_pdf) * S(static_cast<float>(num_lights)),
                    C(0.0)
                    );
            }

            throughput *= src * (dot(n, refl_dir) / brdf_pdf);
            throughput = select(zero_pdf, C(0.0), throughput);

            if (bounce >= 2)
            {
                // Russian roulette
                auto prob = max_element(throughput.samples());
                auto terminate = gen.next() > prob;
                active_rays &= !terminate;
                throughput /= prob;

                if (!any(active_rays))
                {
                    break;
                }
            }

            ray.ori = hit_rec.isect_pos + refl_dir * S(params.epsilon);
            ray.dir = refl_dir;

            last_specular = inter == surface_interaction::SpecularReflection ||
                            inter == surface_interaction::SpecularTransmission;

        }

        result.color = select( result.hit, to_rgba(intensity), result.color );

        return result;
    }

    template <typename R, typename Generator>
    VSNRAY_FUNC result_record<typename R::scalar_type> operator()(
            R ray,
            Generator& gen,
            int x,
            int y
            ) const
    {
        default_intersector ignore;
        return (*this)(ignore, ray, gen, x, y);
    }

    bool instance_color_mode = true;
    std::vector<unsigned> const& geom_id_list;
};

} // visionaray
