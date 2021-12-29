#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Object.h"
#include "Triangle.h"


class Mesh: public Object {
public:
    std::string file_dir;
    std::vector<Vector3f> v;
    std::vector<Vector2f> vt;
    std::vector<Vector3f> vn;
    struct TriangleIndex {
        int x[3][3]; // each row one vertex: v/vt/vn
        int *operator[](const int i) { return x[i]; }
    };
    std::vector<TriangleIndex> f;
    std::vector<std::shared_ptr<Material> > f_materials;

    class BVH {
    public:
        class BVHNode {
        public:
            int id = -1, n_triangles = 0;
            Aabb aabb;
            Triangle* triangles = nullptr;
            BVHNode *child[2];

            BVHNode(){
                child[0] = child[1] = nullptr;
            }
            ~BVHNode(){
                if (triangles != nullptr) delete[] triangles;
            }

            Intersection intersect(const Ray &ray) const {
                // if(id <= 3) std::cout << "Node " << id << ": AABB = (" << aabb.p0 << ", " << aabb.p1 << ")" << std::endl;
                if (aabb.intersect(ray).type == MISS) {
                    // std::cout << "MISS" << std::endl;
                    return Intersection();
                }
                // Intersection ret = triangle.intersect(ray);
                Intersection ret = Intersection();
                if (n_triangles) {
                    for (int i = 0; i < n_triangles; i++) {
                        Intersection tmp = triangles[i].intersect(ray);
                        if (tmp.type != MISS && tmp.t < ret.t) ret = tmp;
                    }
                    return ret;
                }
                for (int i = 0; i < 2; i++)
                    if (child[i] != nullptr) {
                        // std::cout << " entering child[" << i << "]" << std::endl;
                        Intersection tmp = child[i]->intersect(ray);
                        if (tmp.type != MISS && tmp.t < ret.t) ret = tmp;
                    }
                // if (ret.type == MISS) {
                //     std::cout << "Finally MISS" << std::endl;
                // }
                // else {
                //     std::cout << "ret.t = " << ret.t << std::endl;
                // }
                return ret;
            }
        };

        BVHNode *root;

        BVH(){}
        BVH(Mesh *mesh) {
            build(mesh);
        }
        ~BVH(){
            if (root != nullptr) delete_treenode(root);
        }

        class center_less {
        public:
            int axis;
            Vector3f *centers;
            center_less(int _axis, Vector3f *_centers): axis(_axis), centers(_centers) {}
            bool operator()(int a, int b) const {
                if (axis == 0) return centers[a].x < centers[b].x;
                else if (axis == 1) return centers[a].y < centers[b].y;
                else return centers[a].z < centers[b].z;
            }
        };

        void build_node(BVHNode *u, Mesh *mesh, Vector3f *centers, int *ids, int l, int r) {
            std::vector<Mesh::TriangleIndex> &f = mesh->f;
            std::vector<Vector3f> &v = mesh->v;
            Aabb &aabb = u->aabb;
            // compute bounding box
            for (int i = l; i < r; i++) {
                int id = ids[i]; // triangle id
                for (int j = 0; j < 3; j++) {
                    Vector3f p = v[f[id][j][0]];
                    aabb.p0.x = std::min(aabb.p0.x, p.x), aabb.p0.y = std::min(aabb.p0.y, p.y), aabb.p0.z = std::min(aabb.p0.z, p.z);
                    aabb.p1.x = std::max(aabb.p1.x, p.x), aabb.p1.y = std::max(aabb.p1.y, p.y), aabb.p1.z = std::max(aabb.p1.z, p.z);
                }
            }
            if (r - l <= 5) { // leaf if there are <= 5 triangles
                u->n_triangles = r - l;
                // num_tree_triangles += u->n_triangles;
                u->triangles = new Triangle[u->n_triangles];
                for (int i = 0; i < u->n_triangles; i++) {
                    TriangleIndex idx = f[ids[l + i]];
                    u->triangles[i] = Triangle(v[idx[0][0]],  v[idx[1][0]],  v[idx[2][0]],  
                                   mesh->vt[idx[0][1]], mesh->vt[idx[1][1]], mesh->vt[idx[2][1]], 
                                   mesh->vn[idx[0][2]], mesh->vn[idx[1][2]], mesh->vn[idx[2][2]], mesh->f_materials[ids[l + i]]);
                }
            }
            else {
                // find the axis with largest length
                int axis;
                double l0 = aabb.p1.x - aabb.p0.x, l1 = aabb.p1.y - aabb.p0.y, l2 = aabb.p1.z - aabb.p0.z;
                if (l1 > l0) {
                    if (l2 > l1) axis = 2;
                    else axis = 1;
                }
                else {
                    if (l2 > l0) axis = 2;
                    else axis = 0;
                }
                int mid = (l + r) / 2; // index in array "ids"
                std::nth_element(ids + l, ids + mid, ids + r, center_less(axis, centers));
                if (l < mid) {
                    u->child[0] = new BVHNode();
                    u->child[0]->id = 2 * u->id;
                    build_node(u->child[0], mesh, centers, ids, l, mid);
                }
                if (mid + 1 < r) {
                    u->child[1] = new BVHNode();
                    u->child[1]->id = 2 * u->id + 1;
                    build_node(u->child[1], mesh, centers, ids, mid, r);
                }
            }
            // std::cout << "Node " << u->id << ": [" << l << "," << r << ") AABB = (" << aabb.p0 << ", " << aabb.p1 << ") axis = " << axis << std::endl;
        }

        void delete_treenode(BVHNode *u) {
            for (int i = 0; i < 2; i++)
                if (u->child[i] != nullptr)
                    delete_treenode(u->child[i]);
            delete u;
        }

        void build(Mesh *mesh) {
            std::vector<Mesh::TriangleIndex> &f = mesh->f;
            std::vector<Vector3f> &v = mesh->v;
            int num_faces = f.size();
            Vector3f *centers = new Vector3f[num_faces];
            int *ids = new int[num_faces];
            for(int i = 0; i < num_faces; i++) {
                TriangleIndex idx = f[i];
                centers[i] = (v[idx[0][0]] + v[idx[1][0]] + v[idx[2][0]]) * (1.0/3);
                ids[i] = i;
            }
            root = new BVHNode();
            root->id = 1;
            build_node(root, mesh, centers, ids, 0, num_faces);
            delete[] ids;
            delete[] centers;
        }

        Intersection intersect(const Ray &ray) const {
            return root->intersect(ray);
        }
    };
    BVH *bvh;
    
    Mesh(): Object(MESH) {}
    Mesh(std::string fname): Object(MESH, std::shared_ptr<Material>(new Material())) {
        std::cout << "Loading mesh from " << fname << " ..." << std::endl;
        std::ifstream fin(fname.c_str());
        if (fname.find_last_of('/') == std::string::npos) file_dir = "";
        else file_dir = fname.substr(0, fname.find_last_of('/') + 1);
        std::cout << "File directory is " << file_dir << "" << std::endl;
        std::string s;
        std::map<std::string, std::shared_ptr<Material> > materials;
        while (fin >> s) {
            // std::cout << "mesh input, s = " << s << std::endl;
            if (s == "end") break;
            else if (s.length() == 0 || s[0] == '#') {
                fin.ignore(256, '\n');
                continue;
            }
            else if (s == "o" || s == "g" || s == "s" || s == "l") {
                fin.ignore(256, '\n');
                continue;
            }
            else if (s == "mtllib") {
                fin >> s;
                s = file_dir + s;
                std::ifstream mtl_fin(s.c_str());
                std::cout << "Loading materials from " << s << " ..." << std::endl;
                while (mtl_fin >> s) {
                    if (s.length() == 0 || s[0] == '#') {
                        mtl_fin.ignore(256, '\n');
                        continue;
                    }
                    else if (s == "newmtl") {
                        mtl_fin >> s;
                        Material *m = new Material();
                        m->file_dir = file_dir;
                        mtl_fin >> (*m);
                        materials[s] = std::shared_ptr<Material>(m);
                    }
                    else {
                        std::cerr << std::string("Error: Unrecognized MTL file attribute : ") + s << std::endl;
                        exit(-1);
                    }
                }
                // for(auto p: materials) std::cout << p.first << ": " << p.second << ", map_Ke = " << p.second->map_Ke << std::endl;
            }
            else if (s == "v") {
                Vector3f _v;
                fin >> _v;
                v.push_back(_v);
            }
            else if (s == "vt") {
                Vector2f _vt;
                fin >> _vt;
                vt.push_back(_vt);
            }
            else if (s == "vn") {
                Vector3f _vn;
                fin >> _vn;
                vn.push_back(_vn);
            }
            else if (s == "usemtl") {
                fin >> s;
                auto s_material = materials.find(s);
                if (s_material == materials.end()) {
                    std::cerr << std::string("Error: Unrecognized material name : ") + s << std::endl;
                    exit(-1);
                }
                material = materials[s];
            }
            else if (s == "f") {
                TriangleIndex idx;
                for(int i = 0; i < 3; i++) {
                    fin >> s;
                    replace(s.begin(), s.end(), '/', ' ');
                    std::stringstream sstream(s);
                    sstream >> idx[i][0] >> idx[i][1] >> idx[i][2];
                }
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        idx[i][j]--;
                f.push_back(idx);
                f_materials.push_back(material);
            }
            else {
                std::cerr << std::string("Error: Unrecognized object : ") + s << std::endl;
                exit(-1);
            }
        }
        // aabb = new Aabb();
        // for (auto p: v) {
        //     aabb->p0.x = std::min(aabb->p0.x, p.x), aabb->p0.y = std::min(aabb->p0.y, p.y), aabb->p0.z = std::min(aabb->p0.z, p.z);
        //     aabb->p1.x = std::max(aabb->p1.x, p.x), aabb->p1.y = std::max(aabb->p1.y, p.y), aabb->p1.z = std::max(aabb->p1.z, p.z);
        // }
        bvh = new BVH(this);
        std::cout << "Loaded mesh " << fname << ", " << f.size() << " triangles in total." << std::endl;
        // cout << "AABB = (" << aabb->p0 << ", " << aabb->p1 << ")." << std::endl;
        // std::cout << "  BVH root AABB = (" << bvh->root->aabb.p0 << ", " << bvh->root->aabb.p1 << ")." << std::endl;
    }
    ~Mesh() {
        // delete aabb;
        delete bvh;
    }

    Intersection intersect(const Ray &ray) const override {
        // if (aabb->intersect(ray).type == MISS) return Intersection();
        return bvh->intersect(ray);
    }
};
