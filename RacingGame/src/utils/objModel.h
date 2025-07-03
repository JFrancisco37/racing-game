#ifndef OBJMODEL_H
#define OBJMODEL_H

#include <vector>
#include <string>
#include <tiny_obj_loader.h>

struct ObjModel
{
    tinyobj::attrib_t                 attrib;
    std::vector<tinyobj::shape_t>     shapes;
    std::vector<tinyobj::material_t>  materials;

    ObjModel(const char* filename, const char* basepath = NULL, bool triangulate = true);

    void ComputeNormals();
    void BuildTrianglesAndAddToVirtualScene(ObjModel* model);
};


#endif // OBJMODEL_H
