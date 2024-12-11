#version 430

#define PI 3.14159265
#define EPSILON 0.00001
#define INF 10e6

out vec4 FragColor;

struct Ray {
	vec3 origin;
	vec3 direction;
};

// type 0: diffuse, 1: metal, 2: dielectric, 3: emission
struct Material {
    int type;
    vec3 albedo;
    float fuzz;
    float refraction_index;
};

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

uniform Object objects_list[10];
uniform int obj_num;

uniform vec3 cam_pos;
uniform vec3 viewport_bottom_left;
uniform vec3 pixel_delta_x;
uniform vec3 pixel_delta_y;
uniform int max_depth;
uniform int sample_num;

uniform float time;

uniform samplerCube skybox;

float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
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
    
    vec3 outward_normal = (rec.position - center) / radius;
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

vec3 bounce_direction(Material mat, vec3 normal, vec3 incident, Hit_record rec) {
    vec3 rand_unit_vector;
    rand_unit_vector.x = rand(vec2(1, time) + normal.xy + incident.yz);
    rand_unit_vector.y = rand(vec2(2, time) + normal.yz + incident.zx);
    rand_unit_vector.z = rand(vec2(3, time) + normal.zx + incident.xy);
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

bool traverse_objects(Ray ray, out Hit_record rec) {
    bool hit_flag = false;
    float min_hit_t = INF;
    Hit_record tmp_rec;

    for (int i = 0; i < obj_num; i++) {
        if (objects_list[i].type == 0 && sphere_intersect(objects_list[i].position, objects_list[i].radius, objects_list[i].material, ray, tmp_rec)) {
            hit_flag = true;
            if (tmp_rec.t < min_hit_t) {
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

vec3 ray_color(Ray ray) {
    vec3 color = vec3(1.0, 1.0, 1.0);
    vec3 dir = ray.direction;
    vec3 ori = ray.origin;
    
    for (int i = 0; i < max_depth; i++) {
        Hit_record rec;
        if (traverse_objects(ray, rec)) {
            color = color * rec.material.albedo;
            ori = rec.position;
            vec3 new_dir = bounce_direction(rec.material, rec.normal, dir, rec);
            ray = Ray(ori, new_dir);
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
        float rand1 = rand(gl_FragCoord.xy * (time + i)) - 0.5;
        float rand2 = rand(gl_FragCoord.yx * (time - i)) - 0.5;
        vec3 screen_point = viewport_bottom_left + pixel_delta_x * (gl_FragCoord.x + rand1) + pixel_delta_y * (gl_FragCoord.y + rand2);
        Ray ray = Ray(cam_pos, normalize(screen_point - cam_pos));
        color = color + ray_color(ray);
    }

    vec3 output_color;
    output_color.x = sqrt(color.x / sample_num);
    output_color.y = sqrt(color.y / sample_num);
    output_color.z = sqrt(color.z / sample_num);

    //vec3 screen_point = viewport_bottom_left + pixel_delta_x * (gl_FragCoord.x) + pixel_delta_y * (gl_FragCoord.y);
    //Ray ray = Ray(cam_pos, normalize(screen_point - cam_pos));

    FragColor = vec4(output_color, 1.0);
}


