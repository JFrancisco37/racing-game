#pragma once
#include <map>
#include <string>
#include <tiny_obj_loader.h>
#include "renderer.h" // Para SceneObject

void ComputeNormals(ObjModel* model);
void BuildTrianglesAndAddToVirtualScene(ObjModel* model, std::map<std::string, SceneObject>& virtualScene);
