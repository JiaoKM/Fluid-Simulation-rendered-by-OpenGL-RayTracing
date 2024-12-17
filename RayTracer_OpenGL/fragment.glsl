#version 430

#define PI 3.14159265
#define EPSILON 0.0001
#define INF 10e8

out vec4 FragColor;

layout(binding = 2) uniform samplerBuffer objectData;
layout(binding = 3) uniform samplerBuffer BVHData;

struct Ray {
	vec3 origin;
	vec3 direction;
};

// type 0: diffuse, 1: metal, 2: dielectric, 3: emission, 4: texture
struct Material {
    int type;
    vec3 albedo;
    float fuzz;
    float refraction_index;
    
};

// information of intersection
struct Hit_record {
    bool is_intersected;
    vec3 position;
    vec3 normal;
    Material material;
    float t;
    bool front_face;
};

// type 0: sphere, 1: triangle
struct Object {
    int type;
    vec3 position;
    vec3 p1;
    vec3 p2;
    float radius;
    Material material;
};

struct BVH_node {
    int num_objects;
    vec3 min_point;
    vec3 max_point;
    int left_child;
    int right_child;
    int object_index;
};

uniform Object objects_list[5];
uniform int obj_num;

uniform vec3 cam_pos;
uniform vec3 viewport_bottom_left;
uniform vec3 pixel_delta_x;
uniform vec3 pixel_delta_y;
uniform int max_depth;
uniform int sample_num;

uniform float time;
int rand_count = 0;

uniform samplerCube skybox;
uniform sampler2D earthmap;
uniform float rotation_angle;

// random in [0, 1]
float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

float rand_unit() {
    float a = rand(vec2(gl_FragCoord.x, time));
    float b = rand(vec2(time / 4234.8, gl_FragCoord.y));
    float c = rand(vec2(rand_count++, time + 424.23));
    float d = rand(vec2(sqrt(time), a));
    float e = rand(vec2(b, c));
    float f = rand(vec2(d, e));
    
    return f;
}

// read Object and BVH node in texture buffer
Object getObject(int index) {
    Object obj;

    int baseIndex = index * 5; 

    obj.type = int(texelFetch(objectData, baseIndex).x + EPSILON);
    obj.position = vec3(texelFetch(objectData, baseIndex).z, texelFetch(objectData, baseIndex).w, texelFetch(objectData, baseIndex + 1).x);
    obj.p1 = vec3(texelFetch(objectData, baseIndex + 1).y, texelFetch(objectData, baseIndex + 1).z, texelFetch(objectData, baseIndex + 1).w);
    obj.p2 = vec3(texelFetch(objectData, baseIndex + 2).x, texelFetch(objectData, baseIndex + 2).y, texelFetch(objectData, baseIndex + 2).z);
    obj.radius = texelFetch(objectData, baseIndex + 2).w;

    obj.material.type = int(texelFetch(objectData, baseIndex + 3).x + EPSILON);
    obj.material.albedo = vec3(texelFetch(objectData, baseIndex + 3).z, texelFetch(objectData, baseIndex + 3).w, texelFetch(objectData, baseIndex + 4).x);
    obj.material.fuzz = texelFetch(objectData, baseIndex + 4).y;
    obj.material.refraction_index = texelFetch(objectData, baseIndex + 4).z;

    return obj;
}

BVH_node getBVHNode(int index) {
    BVH_node node;

    int baseIndex = index * 3;

    node.num_objects = int(texelFetch(BVHData, baseIndex).x + EPSILON);
    node.min_point = vec3(texelFetch(BVHData, baseIndex).y, texelFetch(BVHData, baseIndex).z, texelFetch(BVHData, baseIndex).w);
    node.max_point = vec3(texelFetch(BVHData, baseIndex + 1).x, texelFetch(BVHData, baseIndex+ 1).y, texelFetch(BVHData, baseIndex + 1).z);
    node.left_child = int(texelFetch(BVHData, baseIndex + 1).w + EPSILON);
    node.right_child = int(texelFetch(BVHData, baseIndex + 2).x + EPSILON);
    node.object_index = int(texelFetch(BVHData, baseIndex + 2).y + EPSILON);

    return node;
}

// calculate intersection with AABB, sphere and triangle
float AABB_intersect(Ray ray, vec3 min_point, vec3 max_point) {
    vec3 inv_dir = 1.0 / ray.direction;
    vec3 t0 = (min_point - ray.origin) * inv_dir;
    vec3 t1 = (max_point - ray.origin) * inv_dir;

    vec3 t_min = min(t0, t1);
    vec3 t_max = max(t0, t1);

    float t_near = max(max(t_min.x, t_min.y), t_min.z);
    float t_far = min(min(t_max.x, t_max.y), t_max.z);

    return (t_far > t_near) ? ((t_near > 0.0) ? (t_near) : (t_far)) : (-1.0);
}

bool sphere_intersect(vec3 center, float radius, Material mat, Ray ray, out Hit_record rec) {
    vec3 oc = center - ray.origin;
    float a = dot(ray.direction, ray.direction);
    float h = dot(ray.direction, oc);
    float c = length(oc) * length(oc) - radius * radius;

    float discriminant = h * h - a * c;
    if (discriminant < 0) {
        rec.is_intersected = false;
        return false;
    }

    float sqrtd = sqrt(discriminant);
    float root = (h - sqrtd) / a;
    if (root < EPSILON || root > INF) {
        root = (h + sqrtd) / a;
        if (root < EPSILON || root > INF) {
            rec.is_intersected = false;
            return false;
        }
    }

    rec.t = root;
    rec.position = ray.origin + root * ray.direction;
    rec.material = mat;

    vec3 outward_normal = normalize((rec.position - center) / radius);
    if (dot(ray.direction, outward_normal) < 0) {
        rec.normal = outward_normal;
        rec.front_face = true;
    }
    else {
        rec.normal = -outward_normal;
        rec.front_face = false;
    }
    
    return true;
}

bool triangle_intersect(vec3 v0, vec3 v1, vec3 v2, Material mat, Ray ray, out Hit_record rec) {
    vec3 n = cross(v1 - v0, v2 - v0);
    float n_norm = sqrt(dot(n, n));
    vec3 normal = normalize(n);

    float m = -dot(normal, v0);
    float denom = dot(normal, ray.direction);

    if (abs(denom) < 0.001) {
        return false;
    }

    float t = (dot(normal, ray.origin) + m) / (-denom);
    if (t < EPSILON || t > INF) {
        return false;
    }

    vec3 pos = ray.origin + t * ray.direction;
    vec3 planar_hitpt_vector = pos - v0;
    vec3 e1xr = cross(v1 - v0, planar_hitpt_vector);
    vec3 e2xr = cross(planar_hitpt_vector, v2 - v0);
    float v = sqrt(dot(e1xr, e1xr)) / n_norm;
    float w = sqrt(dot(e2xr, e2xr)) / n_norm;

    if (dot(e1xr, normal) >= 0 && dot(e2xr, normal) >= 0 && v + w < 1 + EPSILON) {
        rec.t = t;
        rec.material = mat;
        if (dot(ray.direction, normal) < 0) {
            rec.normal = normal;
            rec.front_face = true;
        }
        else {
            rec.normal = -normal;
            rec.front_face = false;
        }
        rec.position = pos;

        return true;
    }

    return false;
}

// calculate the new direction based on the type of material
vec3 bounce_direction(Material mat, vec3 normal, vec3 incident, Hit_record rec) {
    vec3 rand_unit_vector = vec3(rand_unit(), rand_unit(), rand_unit());
    rand_unit_vector = normalize(rand_unit_vector);

    if (mat.type == 0) {
        return normalize(normal + rand_unit_vector);
    }
    else if (mat.type == 1) {
        vec3 reflected = reflect(normalize(incident), normal);
        return normalize(normalize(reflected) + mat.fuzz * rand_unit_vector);
    }
    else if (mat.type == 2) {
        float ri = mat.refraction_index;
        if (rec.front_face) {
            ri = 1.0 / mat.refraction_index;
        }
        float cos_theta = min(dot(-incident, normal), 1.0);
        float sin_theta = sqrt(1 - cos_theta * cos_theta);

        bool cannot_refract = ri * sin_theta > 1.0;
        float r0 = (1.0 - ri) / (1.0 + ri) * (1.0 - ri) / (1.0 + ri);
        float reflectance = r0 + (1 - r0) * pow((1 - cos_theta), 5);

        if (cannot_refract || reflectance > rand(vec2(4, time) + normal.xz + incident.yz)) {
            return normalize(reflect(normalize(incident), normal));
        }
        else {
            return normalize(refract(normalize(incident), normal, ri));
        }
    }
}

// simple traverse
bool traverse_objects(Ray ray, out Hit_record rec) {
    bool hit_flag = false;
    float min_hit_t = INF;
    Hit_record tmp_rec;

    for (int i = 0; i < obj_num; i++) {
        Object obj = getObject(i);
        if (obj.type == 0 && sphere_intersect(obj.position, obj.radius, obj.material, ray, tmp_rec)) {
            hit_flag = true;
            if (tmp_rec.t < min_hit_t && tmp_rec.t > EPSILON) {
                min_hit_t = tmp_rec.t;
                rec.is_intersected = tmp_rec.is_intersected;
                rec.position = tmp_rec.position;
                rec.normal = tmp_rec.normal;
                rec.material = tmp_rec.material;
                rec.front_face = tmp_rec.front_face;
            }
        }
        if (obj.type == 1 && triangle_intersect(obj.position, obj.p1, obj.p2, obj.material, ray, tmp_rec)) {
            hit_flag = true;
            if (tmp_rec.t < min_hit_t && tmp_rec.t > EPSILON) {
                min_hit_t = tmp_rec.t;
                rec.is_intersected = tmp_rec.is_intersected;
                rec.position = tmp_rec.position;
                rec.normal = tmp_rec.normal;
                rec.material = tmp_rec.material;
                rec.front_face = tmp_rec.front_face;
            }
        }
    }

    return hit_flag;
}

// using BVH to traverse
bool traverse_BVH(Ray ray, out Hit_record rec) {
    bool hit_flag = false;
    float min_hit_t = INF;
    Hit_record tmp_rec;

    int stack[256];
    int stack_ptr = 0;

    stack[stack_ptr++] = 0;
    while (stack_ptr > 0) {
        int top = stack[--stack_ptr];
        BVH_node node = getBVHNode(top);

        if (node.num_objects > 0) {
            int start = node.object_index;
            int end = node.object_index + node.num_objects - 1;
            for (int i = start; i <= end; i++) {
                Object obj = getObject(i);
                if (obj.type == 0 && sphere_intersect(obj.position, obj.radius, obj.material, ray, tmp_rec)) {
                    if (tmp_rec.t < min_hit_t && tmp_rec.t > EPSILON) {
                        hit_flag = true;
                        min_hit_t = tmp_rec.t;
                        rec.is_intersected = tmp_rec.is_intersected;
                        rec.position = tmp_rec.position;
                        rec.normal = tmp_rec.normal;
                        rec.material = tmp_rec.material;
                        rec.front_face = tmp_rec.front_face;
                    }
                }
                if (obj.type == 1 && triangle_intersect(obj.position, obj.p1, obj.p2, obj.material, ray, tmp_rec)) {
                    if (tmp_rec.t < min_hit_t && tmp_rec.t > EPSILON) {
                        hit_flag = true;
                        min_hit_t = tmp_rec.t;
                        rec.is_intersected = tmp_rec.is_intersected;
                        rec.position = tmp_rec.position;
                        rec.normal = tmp_rec.normal;
                        rec.material = tmp_rec.material;
                        rec.front_face = tmp_rec.front_face;
                    }
                }
            }
            continue;
        }

        float t_left = INF;
        float t_right = INF;

        if (node.left_child > 0) {
            BVH_node left_node = getBVHNode(node.left_child);
            t_left = AABB_intersect(ray, left_node.min_point, left_node.max_point);
        }
        if (node.right_child > 0) {
            BVH_node right_node = getBVHNode(node.right_child);
            t_right = AABB_intersect(ray, right_node.min_point, right_node.max_point);
        }

        if (t_left > EPSILON && t_right > EPSILON) {
            if (t_left >= t_right) {
                stack[stack_ptr++] = node.left_child;
                stack[stack_ptr++] = node.right_child;
            }
            else {
                stack[stack_ptr++] = node.right_child;
                stack[stack_ptr++] = node.left_child;
            }
        }
        else if (t_left > EPSILON) {
            stack[stack_ptr++] = node.left_child;
        }
        else if (t_right > EPSILON) {
            stack[stack_ptr++] = node.right_child;
        }
    }

    return hit_flag;
}

// calculate color that one ray gets
vec3 ray_color(Ray ray) {
    vec3 color = vec3(1.0, 1.0, 1.0);
    vec3 dir = ray.direction;
    vec3 ori = ray.origin;
    
    for (int i = 0; i < max_depth; i++) {
        Hit_record rec;
        if (traverse_BVH(ray, rec)) {
            if (rec.material.type == 3) {
                color = color * rec.material.albedo;
                break;
            }
            if (rec.material.type == 4) {
                vec2 texcoord;
                texcoord.x = 1 - fract((acos(rec.normal.x / sqrt(pow(rec.normal.x, 2) + pow(rec.normal.z, 2))) + rotation_angle) / (2 * PI));
                texcoord.y = acos(rec.normal.y) / PI;
                color = color * texture(earthmap, texcoord).rgb;
                break;
            }
            color = color * rec.material.albedo;
            ori = rec.position;
            vec3 new_dir = bounce_direction(rec.material, rec.normal, dir, rec);
            ray = Ray(ori + EPSILON * new_dir, normalize(new_dir));
            dir = new_dir;
        }
        else {
            color = color * texture(skybox, normalize(ray.direction)).rgb;
            break;
        }
    }

    return color;
}

void main() {
    vec3 color = vec3(0.0, 0.0, 0.0);

    for (int i = 0; i < sample_num; i++) {
        vec3 screen_point = viewport_bottom_left + pixel_delta_x * (gl_FragCoord.x + rand_unit()) + pixel_delta_y * (gl_FragCoord.y + rand_unit());
        Ray ray = Ray(cam_pos, normalize(screen_point - cam_pos));
        color = color + ray_color(ray);
    }

    vec3 output_color;
    output_color.x = pow(color.x / sample_num, 1.0 / 2.2);
    output_color.y = pow(color.y / sample_num, 1.0 / 2.2);
    output_color.z = pow(color.z / sample_num, 1.0 / 2.2);

    FragColor = vec4(output_color, 1.0);
}


