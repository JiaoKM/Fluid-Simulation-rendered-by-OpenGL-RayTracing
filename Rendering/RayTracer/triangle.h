//
// Created by 16920 on 2024/11/21.
//

#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <complex>
#include <memory>

#include "hittable.h"

class triangle: public hittable {
public:
    triangle(const point3& V0, const point3& V1, const point3& V2, std::shared_ptr<material> mat): V0(V0), V1(V1), V2(V2), mat(mat) {
        auto n = cross(V1 - V0, V2 - V0);
        normal = unit_vector(n);
        n_norm = std::sqrt(dot(n, n));

        set_bounding_box();
    }

    virtual void set_bounding_box() {
        auto edge1 = aabb(V0, V1);
        auto edge2 = aabb(V1, V2);
        auto edge3 = aabb(V2, V0);
        bbox = aabb(edge1, edge2);
        bbox = aabb(bbox, edge3);
    }

    aabb bounding_box() const override { return bbox; }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        float m = -dot(normal, V0);

        auto denom = dot(normal, r.direction());

        if (std::fabs(denom) < 1e-6) return false;

        auto t = (dot(normal, r.origin()) + m) / -denom;
        if (!ray_t.contains(t)) return false;

        auto intersection = r.at(t);
        vec3 planar_hitpt_vector = intersection - V0;
        vec3 e1xr = cross(V1 - V0, planar_hitpt_vector);
        vec3 e2xr = cross(planar_hitpt_vector, V2 - V0);
        float v = std::sqrt(dot(e1xr, e1xr)) / n_norm;
        float w = std::sqrt(dot(e2xr, e2xr)) / n_norm;

        if (dot(e1xr, normal) >= 0 && dot(e2xr, normal) >= 0 && v + w < 1) {
            rec.t = t;
            rec.p = intersection;
            rec.mat = mat;
            rec.set_face_normal(r, normal);

            return true;
        }

        return false;
    }

private:
    point3 V0, V1, V2;
    shared_ptr<material> mat;
    aabb bbox;
    vec3 normal;
    float n_norm;
};

#endif //TRIANGLE_H
