#pragma once

#include <cmath>
#include <vector>
#include <array>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <algorithm>

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include <spdlog/spdlog.h>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "tiny_obj_loader.h"


namespace YamadaMeshFixer{

    using T_NUM = float;
    const T_NUM GLOBAL_TOLERANCE = 1e-6;

    struct Coordinate{
        T_NUM coords[3];

        Coordinate(){
            for (int i = 0; i < 3; i++) {
				coords[i] = 0.0;
            }
        }

        Coordinate(const T_NUM& x, const T_NUM& y, const T_NUM& z){
            coords[0] = x;
            coords[1] = y;
            coords[2] = z;
        }

        Coordinate(const T_NUM* num_list){
            for (int i = 0; i < 3; i++) {
				coords[i] = num_list[i];
            }
        }

        T_NUM& x(){
            return coords[0];
        }

        T_NUM& y(){
            return coords[1];
        }

        T_NUM& z(){
            return coords[2];
        }

        T_NUM& operator[](unsigned int i){
            return coords[i];
        }

		const T_NUM& operator[](unsigned int i) const {
            return coords[i];
        }

        bool operator==(const Coordinate& other_coord) const {
            for(int i=0;i<3;i++){
                if(abs(other_coord[i] - coords[i])>GLOBAL_TOLERANCE ){
                    return false;
                }
            }
            return true;
        }

        Coordinate operator+(const Coordinate& other_coord) const{
            return Coordinate(
                coords[0] + other_coord[0],
                coords[1] + other_coord[1],
                coords[2] + other_coord[2]
            );
        }

        Coordinate operator-(const Coordinate& other_coord) const{
            return Coordinate(
                coords[0] - other_coord[0],
                coords[1] - other_coord[1],
                coords[2] - other_coord[2]
            );
        }

        Coordinate operator*(const T_NUM& k) const{
            return Coordinate(
                coords[0] *k,
                coords[1] *k,
                coords[2] *k
            );
        }

        Coordinate operator/(const T_NUM& k) const{
            return Coordinate(
                coords[0] /k,
                coords[1] /k,
                coords[2] /k
            );
        }

        T_NUM Dot(const Coordinate& other_coord) const{
            T_NUM ans = 0.0;
            for(int i=0;i<3;i++){
                ans += coords[i] * other_coord[i];
            }

            return ans;
        }

        Coordinate Cross(const Coordinate& other_coord) const{
            return Coordinate(
                coords[1]*other_coord[2] - coords[2]*other_coord[1],
                coords[2]*other_coord[0] - coords[0]*other_coord[2],
                coords[0]*other_coord[1] - coords[1]*other_coord[0]
            );
        }
    };

    struct Vertex;
    struct HalfEdge;
    struct Edge;
    struct Loop;
    struct Face;
    struct Solid;
    struct Entity;

    struct Vertex: public Entity{
        Coordinate pointCoord;
    };

    struct HalfEdge: public Entity{
        bool sense;
        std::shared_ptr<HalfEdge> partner;
        std::shared_ptr<Loop> loop;
        std::shared_ptr<HalfEdge> pre, next;
    };

    struct Edge: public Entity{
        std::shared_ptr<Vertex> st, ed;
        std::vector<std::shared_ptr<HalfEdge>> halfEdges; // 不保证顺序
    };

    struct Loop: public Entity{
        std::shared_ptr<HalfEdge> st;
        std::shared_ptr<Face> face;
    };

    struct Face: public Entity{
        std::shared_ptr<Loop> st;
        std::shared_ptr<Loop> solid;
    };

    struct Solid: public Entity{
        std::vector<std::shared_ptr<Loop>> faces;
    };

    struct Entity{};

    struct ObjInfo{
        std::vector<float> vertices;
        std::vector<int> indices;

        std::vector<std::pair<int, int>> solidIndicesRange;

        void LoadFromObj(const std::string& obj_path){
            tinyobj::ObjReader reader;

            if (!reader.ParseFromFile(obj_path)) {
				if (!reader.Error().empty()) {
					SPDLOG_ERROR("TinyObjReader: {}", reader.Error());
				}
				throw std::runtime_error("TinyObjReader read failed.");
			}

            if(!reader.Warning().empty()){
                SPDLOG_WARN("TinyObjReader: {}", reader.Warning());
            }

            auto& attrib = reader.GetAttrib();
			auto& shapes = reader.GetShapes();

            vertices = attrib.vertices; // 复制

            // 遍历每个solid
            int last_end = 0; 
            for(size_t s = 0; s<shapes.size(); s++){
                size_t index_offset = 0;

                // 遍历此solid中的每个face
                for(size_t f = 0; f<shapes[s].mesh.num_face_vertices.size(); f++){
                    size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
                    assert(fv == 3);

                    // 遍历此face中的每个顶点
                    for(size_t v = 0; v < fv; v++){
                        // 取得顶点索引类
                        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                        // 取得顶点索引对应的顶点坐标分量
                        //tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                        //tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                        //tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                        indices.emplace_back(idx.vertex_index);
                    }

                    index_offset += fv;
                }

                solidIndicesRange.emplace_back(last_end, indices.size());
                last_end = indices.size();
            }
        }
    
        Coordinate GetPoint(int index){
            return Coordinate(vertices[3*index+0], vertices[3*index+1], vertices[3*index+2]);
        }

    };

    struct MarkNum{

        int solidCount;
        int faceCount;
        int loopCount;
        int halfedgeCount;
        int edgeCount;
        int vertexCount;

        std::map<Entity*, std::pair<std::string, int>> markNumMap;
        std::map<Entity*, int> solidMap;

        static MarkNum& GetInstance(){
            static MarkNum marknum_instance;
            return marknum_instance;
        }

        // 构造
        void Init(ObjInfo& obj_info){
            Clear();

            // 顶点
            for(int i=0,j=0;i<obj_info.vertices.size();i+=3, j+=1){
                auto vertex_ptr = std::make_shared<Vertex>();
                vertex_ptr->pointCoord = obj_info.GetPoint(j);
                markNumMap[vertex_ptr.get()] = {"Vertex", ++vertexCount};
            }

            // 对于每个solid
            int s = 0;
            for(auto solid_range: obj_info.solidIndicesRange){
                int range_begin = solid_range.first;
                int range_end = solid_range.second;

                for(int j = range_begin; j<range_end;j++){
                    
                }

                s++;
            }

            // for(int i=0;i<obj_info.indices.size();i+=3){
            //     int j = i+1;
            //     int k = i+2;

            //     int index_i = obj_info.indices[i];
            //     int index_j = obj_info.indices[j];
            //     int index_k = obj_info.indices[k];
            // }
        }

        void Clear(){
            markNumMap.clear();
            solidMap.clear();

            solidCount = 0;
            faceCount = 0;
            loopCount = 0;
            halfedgeCount = 0;
            edgeCount = 0;
            vertexCount = 0;
        }
    };

}
