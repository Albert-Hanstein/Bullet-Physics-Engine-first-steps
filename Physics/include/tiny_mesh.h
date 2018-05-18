//
// Copyright 2012-2015, Syoyo Fujita.
//
// Licensed under 2-clause BSD liecense.
//
#ifndef _TINY_MESH_H
#define _TINY_MESH_H

#include <string>
#include <vector>
#include <map>

#include "tiny_obj_loader.h"

class TinyMesh {
public:
TinyMesh(const char * name) {
        answer = tinyobj::LoadObj(shapes, materials, name);
        fname = std::string(name);
}
TinyMesh(const char * name, const char * matpath) {
        answer = tinyobj::LoadObj(shapes, materials, name, matpath);
        fname = std::string(name);
}
char * Name() {
        return (char *) fname.c_str();
}
int Number_Shapes(void) {
        return shapes.size();
}
int Number_Positions(int n) {
        return shapes[n].mesh.positions.size();
}
float * Positions(int n) {
        return shapes[n].mesh.positions.data();
}
int Number_Indices(int n) {
        return shapes[n].mesh.indices.size();
}
unsigned int * Indices(int n) {
        return shapes[n].mesh.indices.data();
}
void DumpData(void) {
        printf("%s\n", Name());
        printf("Shape %d\n", shapes.size());
        printf("Positions mesh %d\n", shapes[0].mesh.positions.size());
        printf("Normals mesh %d\n", shapes[0].mesh.normals.size());
        printf("Texture mesh %d\n", shapes[0].mesh.texcoords.size());
        printf("Indices mesh %d\n", shapes[0].mesh.indices.size());
        printf("Material mesh %d\n", shapes[0].mesh.material_ids.size());

}
void DumpData(int i) {
        printf("Shape %d\n", i);
        printf("Positions mesh %d (%d)\n", shapes[i].mesh.positions.size(), shapes[i].mesh.positions.size()/3);
        printf("Normals mesh %d (%d)\n", shapes[i].mesh.normals.size(), shapes[i].mesh.normals.size()/3);
        printf("Texture mesh %d (%d)\n", shapes[i].mesh.texcoords.size(), shapes[i].mesh.texcoords.size()/2);
        printf("Indices mesh %d\n", shapes[i].mesh.indices.size());
        printf("Material mesh id %d\n", shapes[i].mesh.material_ids.size());
        if(materials.size() > 0) {
                tinyobj::material_t m = materials.at(i);
                printf("Maerial %s\n",  m.name.c_str());
        }
        else{
                printf("No materials\n");
        }
}
~TinyMesh() {
};
std::vector<tinyobj::shape_t> shapes;
std::vector<tinyobj::material_t> materials;
private:
std::string fname, answer;
};

#endif // _TINY_OBJ_LOADER_H
