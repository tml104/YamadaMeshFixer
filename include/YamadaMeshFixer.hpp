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
#include <functional>
#include <list>
#include <queue>

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include "SpdlogDef.hpp"
#include <spdlog/spdlog.h>

#include "tiny_obj_loader.h"


namespace YamadaMeshFixer{


    using T_NUM = double;
    const T_NUM GLOBAL_TOLERANCE = 1e-12;

    const T_NUM EPSLION1 = 0.05;
    const T_NUM EPSLION2 = cos(M_PI / 6);

    const T_NUM EPSLION2_SIN = sqrt(1.0 - EPSLION2 * EPSLION2);

    const T_NUM PARAM_EPSLION = 0.05; // 用于分割时配对点用的
    const T_NUM VOLUME_PROPORTION_THRESHOLD = 0.9;

  	const T_NUM MINVAL = -1e9;
	const T_NUM MAXVAL = 1e9;

    namespace Utils{

        auto SplitStr(const std::string &s, char split_char = ',')
        {
            std::vector<std::string> res;
            //lambda
            auto string_find_first_not = [](const std::string &s, size_t pos = 0, char split_char = ',') {
                for (size_t i = pos; i < s.size(); i++)
                {
                    if (s[i] != split_char && s[i]!=' ' && s[i]!='\t')
                        return i;
                }
                return std::string::npos;
            };

            size_t begin_pos = string_find_first_not(s, 0, split_char);
            size_t end_pos = s.find(split_char, begin_pos);

            while (begin_pos != std::string::npos)
            {
                size_t end_pos2=end_pos-1;
                while(begin_pos<end_pos2 && (s[end_pos2]=='\t' || s[end_pos2]==' '))
                {
                    end_pos2--;
                }
                res.emplace_back(s.substr(begin_pos, end_pos2 +1 - begin_pos));
                begin_pos = string_find_first_not(s, end_pos, split_char);
                end_pos = s.find(split_char, begin_pos);
            }
            return res;
        }

        template <typename T>
        T clamp(T value, T l, T r){
            if(value<l) return l;
            if(value>r) return r;
            return value;
        }

    }

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
                if(std::abs(other_coord[i] - coords[i])>GLOBAL_TOLERANCE ){
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

        Coordinate Min(const Coordinate& other_coord) const {
            return Coordinate(
                std::min(coords[0], other_coord[0]),
                std::min(coords[1], other_coord[1]),
                std::min(coords[2], other_coord[2])
            );
        }

        Coordinate Max(const Coordinate& other_coord) const {
            return Coordinate(
                std::max(coords[0], other_coord[0]),
                std::max(coords[1], other_coord[1]),
                std::max(coords[2], other_coord[2])
            );
        }

        T_NUM Distance(const Coordinate& other_coord) const {
            T_NUM dis = 0.0;
            for(int i=0;i<3;i++){
                dis += (coords[i] - other_coord[i]) * (coords[i] - other_coord[i]);
            }

            return sqrt(dis);
        }

        T_NUM Length() const {
            T_NUM length = 0.0;

            for(int i=0;i<3;i++){
                length += (coords[i] * coords[i]);
            }

            return sqrt(length);
        }

        void Normalize(){
            T_NUM length = Length();

            for(int i=0;i<3;i++){
                coords[i] /= length;
            }
        }

        T_NUM Volume(const Coordinate& other_coord) const {
            T_NUM x_length = std::abs(coords[0] - other_coord[0]);
            T_NUM y_length = std::abs(coords[1] - other_coord[1]);
            T_NUM z_length = std::abs(coords[2] - other_coord[2]);

            return x_length * y_length * z_length;
        }
    };

    struct Entity{};

    struct Vertex;
    struct HalfEdge;
    struct Edge;
    struct Loop;
    struct Face;
    struct Solid;

    struct Vertex: public Entity{
        Coordinate pointCoord;
    };

    struct Edge: public Entity{
        std::shared_ptr<Vertex> st, ed;
        std::vector<std::shared_ptr<HalfEdge>> halfEdges; // 不保证顺序

        T_NUM Length() const {
            return st->pointCoord.Distance(ed->pointCoord);
        }

        void AddHalfEdge(const std::shared_ptr<HalfEdge>& he){
            halfEdges.emplace_back(he);
        }

        void UpdateHalfEdgesPartner();
    };

    struct HalfEdge: public Entity{
        // false: 与edge一致, true: 不一致
        bool sense; 
        std::shared_ptr<HalfEdge> partner;
        std::shared_ptr<Loop> loop;
        std::shared_ptr<HalfEdge> pre, next;
        std::shared_ptr<Edge> edge;

        std::shared_ptr<Vertex> GetStart() const {
            return sense?(edge->ed):(edge->st);
        }

        std::shared_ptr<Vertex> GetEnd() const {
            return sense?(edge->st):(edge->ed);
        }

        T_NUM Length() const{
            return edge->Length();
        }
    };

    struct Loop: public Entity{
        std::shared_ptr<HalfEdge> st;
        std::shared_ptr<Face> face;
    };

    struct Face: public Entity{
        std::shared_ptr<Loop> st;
        std::shared_ptr<Solid> solid;
    };

    struct Solid: public Entity{
        std::set<std::shared_ptr<Face>> faces;

        void AddFace(const std::shared_ptr<Face>& f){
            faces.insert(f);
        }

        void RemoveFace(const std::shared_ptr<Face>& f){
            if(auto it = faces.find(f); it != faces.end()){
                faces.erase(it);
            }
        }
    };

    void Edge::UpdateHalfEdgesPartner(){
        // 按照halfedge添加顺序串成一个环
        for(int h=0;h<halfEdges.size();h++){
            halfEdges[h]->partner = halfEdges[(h+1) % halfEdges.size()];
        }
    }

    struct ObjInfo{
        std::vector<tinyobj::real_t> vertices;
        std::vector<int> indices;

        std::vector<std::pair<int, int>> solidIndicesRange;

        void LoadFromObj(const std::string& obj_path){
            SPDLOG_INFO("Loading OBJ from: {}", obj_path);

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

            SPDLOG_INFO("Loading OBJ done.");
        }
    
        Coordinate GetPoint(int index){
            return Coordinate(vertices[3*index+0], vertices[3*index+1], vertices[3*index+2]);
        }

    };

    enum class TopoType{
        NoExist = 0,
        Entity = 1,
        Solid = 2,
        Face = 3,
        Loop = 4,
        HalfEdge = 5,
        Edge = 6,
        Vertex = 7
    };

    struct MarkNum{

    public:

        // 对应类型最大容量计数
        // int entityCapacity;
        // int solidCapacity;
        // int faceCapacy;
        // int loopCapac;
        // int halfedgeCount;
        // int edgeCount;
        // int vertexCount;

        // 固有的数据结构

        std::map<TopoType, int> capacities;// 对应类型最大容量计数

        std::map<Entity*, std::pair<TopoType, int>> markNumMap; // 用于由指针访问类型与对应计数
        std::vector<std::shared_ptr<Solid>> solids; // 用于访问所有实体

        // std::map<TopoType, std::map<int, std::shared_ptr<Entity>>> idMap; // (TopoType, id) -> Entity*
        
        std::map<TopoType, std::list<int>> deletedIdListsMap;

        // 有特殊用处需要维护的数据结构
        std::map<std::pair<int, int>, std::shared_ptr<Edge>> edgesMap; // (vertex id, vertex id（有序对）) -> edge，注意：索引是有序对！
        
        // 单例
        static MarkNum& GetInstance(){
            static MarkNum marknum_instance;
            return marknum_instance;
        }

        // 由ObjInfo加载半边数据结构
        void LoadFromObjInfo(ObjInfo& obj_info){
            SPDLOG_INFO("Start.");

            Clear();

            // 顶点
            std::vector<std::shared_ptr<Vertex>> vertex_ptrs; // vertices: 顶点列表（顶点导出不能依赖于这个列表）

            for(int i=0,j=0;i<obj_info.vertices.size();i+=3, j+=1){
                auto vertex_ptr = std::make_shared<Vertex>();
                UpdateMarkNumMap(vertex_ptr);
                vertex_ptr->pointCoord = obj_info.GetPoint(j);

                vertex_ptrs.emplace_back(vertex_ptr);
            }

            auto make_edge = [&](int i, int j) -> std::shared_ptr<Edge> {
                
                // 注意：这里只有map的key是需要保证有序的，而下面构造edge的时候会用到原始顺序的索引，因此这里要单独使用变量保存key
                int search_i = i;
                int search_j = j;

                if(search_i>search_j){
                    std::swap(search_i,search_j);
                }

                // 不存在：创建新边
                if(auto it = edgesMap.find({search_i,search_j}); it == edgesMap.end()){
                    std::shared_ptr<Edge> edge_ptr = std::make_shared<Edge>();
                    UpdateMarkNumMap(edge_ptr);

                    // 初始化边的信息（到时候可能要挪到函数里面）
                    // 此处必须使用原始顶点索引
                    edge_ptr->st = vertex_ptrs[i];
                    edge_ptr->ed = vertex_ptrs[j];

                    edgesMap[{search_i,search_j}] = edge_ptr;

                    return edge_ptr;
                }
                // 存在：返回
                else{
                    return it->second;
                }
            };

            auto make_halfedge = [&](int i, int j) -> std::shared_ptr<HalfEdge> {
                auto edge_ptr = make_edge(i,j);
                std::shared_ptr<HalfEdge> halfedge_ptr = std::make_shared<HalfEdge>();
                UpdateMarkNumMap(halfedge_ptr);

                // 更新halfedge的edge
                halfedge_ptr->edge = edge_ptr;

                // 更新sense
                if(markNumMap[edge_ptr->st.get()].second == i && markNumMap[edge_ptr->ed.get()].second == j){
                    halfedge_ptr->sense = false;
                }else{
                    halfedge_ptr->sense = true;
                }
                
                // 添加此halfedge到edge的halfedgeList中，同时更新halfedgeList中已经有的halfedge的相邻关系
                edge_ptr->halfEdges.emplace_back(halfedge_ptr);

                // 按照halfedge添加顺序串成一个环
                edge_ptr->UpdateHalfEdgesPartner();

                return halfedge_ptr;
            };

            auto make_loop = [&](const std::shared_ptr<HalfEdge>& a, const std::shared_ptr<HalfEdge>& b, const std::shared_ptr<HalfEdge>& c) -> std::shared_ptr<Loop>{

                // set next & pre
                a->next = b;
                a->pre = c;

                b->next = c;
                b->pre = a;

                c->next = a;
                c->pre = b;

                // make loop
                std::shared_ptr<Loop> lp = std::make_shared<Loop>();
                UpdateMarkNumMap(lp);
                lp->st = a;

                // set 
                a->loop = lp;
                b->loop = lp;
                c->loop = lp;

                return lp;
            };

            auto make_face = [&](const std::shared_ptr<Loop>& lp) -> std::shared_ptr<Face>{
                // make face
                std::shared_ptr<Face> f = std::make_shared<Face>();
                UpdateMarkNumMap(f);

                f->st = lp;

                // set lp face
                lp->face = f;

                return f;
            };

            auto make_solid = [&](const std::set<std::shared_ptr<Face>>& faces) -> std::shared_ptr<Solid>{

                // make solid
                std::shared_ptr<Solid> solid = std::make_shared<Solid>();
                UpdateMarkNumMap(solid);

                // update face solid
                for(auto f: faces){
                    f->solid = solid;
                }

                // copy faces vector
                solid->faces = faces;

                return solid;
            };

            // 对于每个solid
            int s = 0;
            for(auto solid_range: obj_info.solidIndicesRange){
                int range_begin = solid_range.first;
                int range_end = solid_range.second;

                std::set<std::shared_ptr<Face>> faces;
                // 对应solid的顶点范围: 每3个点的索引构成一个三角形
                for(int i = range_begin; i<range_end; i+=3){
                    int j = i+1;
                    int k = i+2;

                    int index_i = obj_info.indices[i];
                    int index_j = obj_info.indices[j];
                    int index_k = obj_info.indices[k];

                    // half edge
                    auto halfedge_ij = make_halfedge(index_i, index_j);
                    auto halfedge_jk = make_halfedge(index_j, index_k);
                    auto halfedge_ki = make_halfedge(index_k, index_i);

                    // loop
                    auto loop = make_loop(halfedge_ij, halfedge_jk, halfedge_ki);
                    
                    // face
                    auto face = make_face(loop);
                    faces.insert(face);
                }

                // solid
                auto solid = make_solid(faces);
                solids.emplace_back(solid);

                s++;
            }

            SPDLOG_INFO("End.");
        }

        std::shared_ptr<Edge> FindEdgeBetweenVertices(const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2){

            int v1_id = -1, v2_id = -1;
            if(auto it = markNumMap.find(v1.get()); it != markNumMap.end()){
                v1_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("v1 is not exist in markNumMap.");
                return nullptr;
            }

            if(auto it = markNumMap.find(v2.get()); it != markNumMap.end()){
                v2_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("v2 is not exist in markNumMap.");
                return nullptr;
            }

            if(auto v1_v2_edge_it = edgesMap.find({v1_id, v2_id}); v1_v2_edge_it != edgesMap.end()){
                return v1_v2_edge_it->second;
            }

            if(auto v2_v1_edge_it = edgesMap.find({v2_id, v1_id}); v2_v1_edge_it != edgesMap.end()){
                return v2_v1_edge_it->second;
            }
            
            return nullptr;
        }

        // 测试
        void Test(){
            SPDLOG_INFO("Start.");

            SPDLOG_INFO("Entity Capacity: {}", capacities[TopoType::Entity]);
            SPDLOG_INFO("Solid Capacity: {}", capacities[TopoType::Solid]);
            SPDLOG_INFO("Face Capacity: {}", capacities[TopoType::Face]);
            SPDLOG_INFO("Loop Capacity: {}", capacities[TopoType::Loop]);
            SPDLOG_INFO("HalfEdge Capacity: {}", capacities[TopoType::HalfEdge]);
            SPDLOG_INFO("Edge Capacity: {}", capacities[TopoType::Edge]);
            SPDLOG_INFO("Vertex Capacity: {}", capacities[TopoType::Vertex]);

            // 打印被删除元素的链表长度
            for(auto p: deletedIdListsMap){
                auto t = p.first;
                auto l = p.second;
                SPDLOG_INFO("List size: {} {}", static_cast<int>(t), l.size());
            }

            for(auto solid: solids){

                int solid_id = GetId(solid);
                SPDLOG_DEBUG("solid_id: {}", solid_id);

                for(auto face: solid->faces){
                    auto lp = face->st;
                    auto i_half_edge = lp->st;

                    int lp_id = GetId(lp);

                    SPDLOG_DEBUG("lp_id: {}", lp_id);

                    do{
                        if(i_half_edge == nullptr){
                            SPDLOG_ERROR("i_half_edge is nullptr");
                            break;
                        }
                        auto e = i_half_edge->edge;

                        int i_half_edge_id = GetId(i_half_edge);
                        SPDLOG_DEBUG("i_half_edge_id: {}", i_half_edge_id);

                        int e_id = GetId(e);
                        SPDLOG_DEBUG("e_id: {}", e_id);

                        auto e_st = e->st;
                        auto e_ed = e->ed;
                        int e_st_id = GetId(e_st);
                        int e_ed_id = GetId(e_ed);

                        SPDLOG_DEBUG("e_st_id: {}", e_st_id);
                        SPDLOG_DEBUG("e_ed_id: {}", e_ed_id);


                        i_half_edge = i_half_edge->next;

                    }while(i_half_edge && i_half_edge != lp->st);
                }
            }

            SPDLOG_INFO("End.");
        }

        // 添加元素并维护marker的下标
        // 如果添加的是边的话还要顺带维护一下edgesMap
        // 如果输入不带类型的话会很难搞，所以这里还是弄成模板类

        template <typename T>
        void AddEntity(const std::shared_ptr<T>& new_entity){
            auto topotype_name = GetTypeFromTemplate(new_entity);

            int new_id;
            // 优先从deletedIdLists中分配id
            auto& deleted_id_list = deletedIdListsMap[topotype_name];
            if(deleted_id_list.size()){
                new_id = deleted_id_list.front();
                deleted_id_list.pop_front();
            }
            else{ // 否则新分配一个id
                new_id = (capacities[topotype_name]++);
            }

            markNumMap[new_entity.get()] = {topotype_name, new_id};

            // 如果添加的是边，那么维护一下edgesMap
            if constexpr(std::is_same_v<T, Edge>){
                AddEdgeToEdgesMap(new_entity);
            }
        }

        void AddEdgeToEdgesMap(const std::shared_ptr<Edge>& new_entity){
            auto st = new_entity->st;
            auto ed = new_entity->ed;
            int st_id = -1;
            int ed_id = -1;

            // st
            if(auto it = markNumMap.find(st.get()); it != markNumMap.end()){
                st_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("The st of the edge you try to add is no exist in markNumMap.");
            }

            // ed
            if(auto it = markNumMap.find(ed.get()); it != markNumMap.end()){
                ed_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("The ed of the edge you try to add is no exist in markNumMap.");
            }

            if(st_id != -1 && ed_id != -1){
            
                if(st_id > ed_id){
                    std::swap(st_id, ed_id);
                }

                edgesMap[{st_id, ed_id}] = new_entity;
                SPDLOG_DEBUG("edge added: {}, {}", st_id, ed_id);
            }
        }


        // 删除元素
        template <typename T>
        void RemoveEntity(const std::shared_ptr<T>& removed_entity, bool remove_edge_from_edges_map = true){
            auto topotype_name = GetTypeFromTemplate(removed_entity);

            int deleted_id;
            if(auto it = markNumMap.find(removed_entity.get()); it != markNumMap.end()){
                deleted_id = it->second.second;
                // 把deleted_id插入链表中，然后从markNumMap中移除之
                deletedIdListsMap[topotype_name].emplace_back(deleted_id);
                markNumMap.erase(it);
            }
            else{ // 不存在：报错
                SPDLOG_ERROR("The entity you are trying to remove is not exist in markNumMap.");
                return ;
            }

            if constexpr(std::is_same_v<T, Edge>){
                if(remove_edge_from_edges_map) RemoveEdgeFromEdgesMap(removed_entity);
            }
        }

        void RemoveEdgeFromEdgesMap(const std::shared_ptr<Edge>& new_entity){

            auto st = new_entity->st;
            auto ed = new_entity->ed;
            int st_id = -1;
            int ed_id = -1;

            // st
            if(auto it = markNumMap.find(st.get()); it != markNumMap.end()){
                st_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("The st of the edge you try to remove is no exist in markNumMap.");
            }

            // ed
            if(auto it = markNumMap.find(ed.get()); it != markNumMap.end()){
                ed_id = it->second.second;
            }
            else{
                SPDLOG_ERROR("The ed of the edge you try to remove is no exist in markNumMap.");
            }

            if(st_id>ed_id){
                std::swap(st_id, ed_id);
            }

            if(auto it = edgesMap.find({st_id, ed_id}); it != edgesMap.end()){
                edgesMap.erase(it);
                SPDLOG_DEBUG("edge removed: {} {}", st_id, ed_id);
            }
            else{
                SPDLOG_ERROR("The edge you try to remove is no exist in markNumMap: {} {}", st_id, ed_id);
            }
        }

        void ExportSolidsToOBJ(std::string output_path){
            SPDLOG_INFO("Start.");

            auto output_path_splited = Utils::SplitStr(output_path, '/');
            std::string prefix;
            for(int i=0;i<output_path_splited.size()-1;i++){
                prefix += output_path_splited[i] + "/";
            }

            auto output_path_splited2 = Utils::SplitStr(output_path_splited.back(), '.');

            int num = 0;
            for(auto solid: solids){
                Export(solid, prefix+ output_path_splited2[0] + std::to_string(num++) + "." + output_path_splited2[1]);
            }

            SPDLOG_INFO("End.");
        }

        // 导出
        // 这里（应该只）有一种解决方法：自己手写一个导出器
        void Export(const std::shared_ptr<Solid>& solid, std::string output_path){
            SPDLOG_INFO("output_path: {}", output_path);

            std::fstream f;
            f.open(output_path, std::ios::out | std::ios::trunc);

            if(!f.is_open()){
                throw std::runtime_error("Open output_path failed");
            }

            // v
            // for(auto vertex_ptr: vertex_ptrs){
            //     f << "v " << vertex_ptr->pointCoord.x() << " "<< vertex_ptr->pointCoord.y() << " " << vertex_ptr->pointCoord.z() << "\n";
            // }

            // 现在改成全部遍历一遍solid
            int vertex_count = 0;
            std::map<Vertex*, int> vertex_id_map;

            auto process_vertex = [&](const std::shared_ptr<Vertex>& vertex_ptr, const std::shared_ptr<Edge>& e){
                if(auto it = vertex_id_map.find(vertex_ptr.get()); it == vertex_id_map.end()){ // 找不到？加入！
                    vertex_id_map[vertex_ptr.get()] = vertex_count++;

                    if(auto it2 = markNumMap.find(vertex_ptr.get()); it2 == markNumMap.end()){
                        SPDLOG_ERROR("vertex_ptr out of markNumMap!");
                        // auto e_id = markNumMap[e.get()].second;
                        int e_id = GetId(e);
                        SPDLOG_DEBUG("e_id: {}", e_id);

                    }

                    f << "v " << vertex_ptr->pointCoord.x() << " "<< vertex_ptr->pointCoord.y() << " " << vertex_ptr->pointCoord.z() << "\n";
                }
            };


            for(auto face: solid->faces){
                auto lp = face->st;

                auto i_half_edge = lp->st;
                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_ERROR("i_half_edge is nullptr");
                        break;
                    }
                    
                    auto e = i_half_edge->edge;
                    process_vertex(i_half_edge->GetStart(), e);
                    process_vertex(i_half_edge->GetEnd(), e);


                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);
            }

            SPDLOG_DEBUG("vertex_count: {}", vertex_count);

            // f
            for(auto face: solid->faces){
                auto lp = face->st;
                std::vector<int> vertices_3_indices;
                vertices_3_indices.reserve(3);

                auto i_half_edge = lp->st;
                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_ERROR("i_half_edge is nullptr");
                        break;
                    }

                    vertices_3_indices.emplace_back(vertex_id_map[i_half_edge->GetStart().get()]);

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);

                // 输出
                f << "f";
                for(auto vertices_index: vertices_3_indices){
                    f<<" "<<vertices_index + 1;
                }
                f<<"\n";
            }

            f.close();
        }

        // TODO: 刷新MarkNum
        void Refresh(){
        
        }

        void Clear(){
            capacities.clear();
            markNumMap.clear();
            solids.clear();
            deletedIdListsMap.clear();

            edgesMap.clear();

        }

        int GetId(const std::shared_ptr<Entity>& p){
            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.second;
            }

            return -1;
        }

        TopoType GetType(const std::shared_ptr<Entity>& p){

            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.first;
            }

            return TopoType::NoExist;
        }

        template<typename T>
        TopoType GetTypeFromTemplate(const std::shared_ptr<T>& p){
            TopoType topotype_name = TopoType::NoExist;

            if constexpr (std::is_same_v<T, Vertex>){
                topotype_name = TopoType::Vertex;
            }
            else if constexpr (std::is_same_v<T, HalfEdge>){
                topotype_name = TopoType::HalfEdge;
            }
            else if constexpr (std::is_same_v<T, Edge>){
                topotype_name = TopoType::Edge;
            }
            else if constexpr (std::is_same_v<T, Loop>){
                topotype_name = TopoType::Loop;
            }
            else if constexpr (std::is_same_v<T, Face>){
                topotype_name = TopoType::Face;
            }
            else if constexpr (std::is_same_v<T, Solid>){
                topotype_name = TopoType::Solid;
            }
            else{
                topotype_name = TopoType::NoExist;
            }

            return topotype_name;
        }


    private:

        MarkNum(){}

        // 由指针类型更新UpdateMarkNumMap
        // 更新逻辑
        template<typename T>
        void UpdateMarkNumMap(const std::shared_ptr<T>& ptr){
            TopoType topotype_name = GetTypeFromTemplate(ptr);

            markNumMap[ptr.get()] = {topotype_name, (capacities[topotype_name])++};
        }
    };


    namespace GeometryUtils{
        bool GeometryCoincidentVertex(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2){
            if(v1 == nullptr || v2 == nullptr){
                return false;
            }

            if(v1 == v2){
                return true;
            }

            // 判断几何相同
            if(v1->pointCoord == v2->pointCoord){
                return true;
            }

            return false;
        }

        bool GeometryCoincidentEdge(std::shared_ptr<Edge> e1, std::shared_ptr<Edge> e2){

            if(e1==nullptr || e2==nullptr){
                return false;
            }

            // bool same_dir = false;

            // 原来这里要采样，但是这里因为全是直线所以可以直接判断首尾顶点是否相同
            if(GeometryCoincidentVertex(e1->st, e2->st) && GeometryCoincidentVertex(e1->ed, e2->ed)){
                // same_dir = true;
                return true;
            }
            else if(GeometryCoincidentVertex(e1->st, e2->ed) && GeometryCoincidentVertex(e2->st, e1->ed)){
                // same_dir = false;
                return true;
            }
            else{
                return false;
            }

            return false;
        }

        int HalfEdgePartnerCount(const std::shared_ptr<HalfEdge>& he){
            std::shared_ptr<HalfEdge> i_half_edge = he;
            int cnt = 0;

            do{
                
                if(i_half_edge == nullptr){
                    SPDLOG_ERROR("i_half_edge is nullptr");
                    cnt-=1;
                    break;
                }
                cnt++;

                i_half_edge = i_half_edge->partner;
            }while(i_half_edge && i_half_edge != he);

            return cnt;
        }

        int EdgePartnerCount(const std::shared_ptr<Edge>& e){

            if(e == nullptr){
                SPDLOG_ERROR("e is nullptr");
                return 0;
            }

            return e->halfEdges.size();
        }
    
        // 返回值：first是 st~mid 的部分，second是 mid~ed的部分
        std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>> SplitEdge(std::shared_ptr<Edge> e, T_NUM param){

            param = Utils::clamp(param, 0.0, 1.0);

            if(e == nullptr){
                SPDLOG_ERROR("split edge is nullptr");
                return {nullptr, nullptr};
            }

            // 新增元素列表
            // 点：v_c
            // 边：e1, e2, e_h
            // 半边： 由e的每一个i_half_edge分裂而成的he1, he2，新加边的两个半边he_h_l, he_h_r
            // 面和环：he1、he2对应的面和环（每个原先的面生成2个）
            std::vector<std::shared_ptr<Edge>> edgesToBeAdded;
            std::vector<std::shared_ptr<HalfEdge>> halfEdgesToBeAdded;
            std::vector<std::shared_ptr<Loop>> loopsToBeAdded;
            std::vector<std::shared_ptr<Face>> facesToBeAdded;

            // 被删除元素列表
            // 点：-
            // 边：e
            // 半边：e的每一个i_half_edge
            // 面和环：e的每一个i_half_edge对应的每一个面和环
            std::vector<std::shared_ptr<HalfEdge>> halfEdgesToBeDeleted;
            std::vector<std::shared_ptr<Loop>> loopsToBeDeleted;
            std::vector<std::shared_ptr<Face>> facesToBeDeleted;

            // 1. 一次性要创建的新元素
            // 1.1 新顶点Vc
            
            auto v_c = std::make_shared<Vertex>();
            v_c->pointCoord = Coordinate((e->st->pointCoord) + (e->ed->pointCoord - e->st->pointCoord) * param);

            // 1.2 新边e1, e2（维护最后再说）
            auto e1 = std::make_shared<Edge>();
            auto e2 = std::make_shared<Edge>();
            e1->st = e->st;
            e1->ed = v_c;

            e2->st = v_c;
            e2->ed = e->ed;

            edgesToBeAdded.emplace_back(e1);
            edgesToBeAdded.emplace_back(e2);

            // 2. 对每个面，先新建新元素
            for(auto i_half_edge: e->halfEdges){ // 每个he在这里都对应一个面
                auto i_lp = i_half_edge->loop;
                auto i_face = i_lp->face;
                auto i_solid = i_face->solid;

                // 2.1 T1, T2（连接关系稍后维护）
                auto l1 = std::make_shared<Loop>();
                auto f1 = std::make_shared<Face>();
                auto l2 = std::make_shared<Loop>();
                auto f2 = std::make_shared<Face>();

                f1->st = l1;
                l1->face = f1;
                f2->st = l2;
                l2->face = f2;

                // face->solid
                f1->solid = f2->solid = i_solid;

                // 2.2 HE1, HE2（连接关系稍后维护）
                auto he1 = std::make_shared<HalfEdge>();
                auto he2 = std::make_shared<HalfEdge>();

                // -> HE1, HE2的部分内容设置
                he1->edge = e1;
                he2->edge = e2;
                he1->sense = i_half_edge->sense;
                he2->sense = i_half_edge->sense;

                e1->AddHalfEdge(he1);
                e2->AddHalfEdge(he2);

                // 2.3 拿到此半边对应的对顶点Vd，并据此构造eh（从Vd指向Vc）, 以及对应半边

                auto i_half_edge_next = i_half_edge->next;
                auto i_half_edge_pre = i_half_edge->pre;

                auto v_d = i_half_edge_next->GetEnd();
                auto e_h = std::make_shared<Edge>();
                e_h->st = v_d;
                e_h->ed = v_c;

                auto he_h_l = std::make_shared<HalfEdge>();
                auto he_h_r = std::make_shared<HalfEdge>();

                he_h_l->edge = he_h_r->edge = e_h;
                he_h_l->sense = true;
                he_h_r->sense = false;

                e_h->AddHalfEdge(he_h_l);
                e_h->AddHalfEdge(he_h_r);
                e_h->UpdateHalfEdgesPartner();

                // 2.4 维护各个halfedge和T1, T2的连接关系
                l1->st = he1; // 随意指定，这里就指定成新分裂边吧，下同
                l2->st = he2;

                // -> T1的3个halfedge循环
                i_half_edge_pre->pre = he_h_l;
                i_half_edge_pre->next = he1;

                he1->pre = i_half_edge_pre;
                he1->next = he_h_l;

                he_h_l->pre = he1;
                he_h_l->next = i_half_edge_pre;

                i_half_edge_pre->loop = he1->loop = he_h_l->loop = l1;

                // -> T2的3个halfedge循环
                i_half_edge_next->pre = he2;
                i_half_edge_next->next = he_h_r;

                he2->pre = he_h_r;
                he2->next = i_half_edge_next;

                he_h_r->pre = i_half_edge_next;
                he_h_r->next = he2;

                i_half_edge_next->loop = he2->loop = he_h_r->loop = l2;

                // 把f1, f2加入solid中
                i_solid->AddFace(f1);
                i_solid->AddFace(f2);

                i_solid->RemoveFace(i_face);

                // 添加新增和被删除元素
                edgesToBeAdded.emplace_back(e_h);
                halfEdgesToBeAdded.emplace_back(he1);
                halfEdgesToBeAdded.emplace_back(he2);
                halfEdgesToBeAdded.emplace_back(he_h_l);
                halfEdgesToBeAdded.emplace_back(he_h_r);
                loopsToBeAdded.emplace_back(l1);
                loopsToBeAdded.emplace_back(l2);
                facesToBeAdded.emplace_back(f1);
                facesToBeAdded.emplace_back(f2);

                halfEdgesToBeDeleted.emplace_back(i_half_edge);
                loopsToBeDeleted.emplace_back(i_lp);
                facesToBeDeleted.emplace_back(i_face);
            }

            e1->UpdateHalfEdgesPartner();
            e2->UpdateHalfEdgesPartner();
            
            // 注意顺序：删除元素在添加元素之前，这样使得MarkNum最大限度地将被删除的编号重新分配
            // 删除的元素
            MarkNum::GetInstance().RemoveEntity(e);
            for(auto half_edge_to_be_deleted: halfEdgesToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(half_edge_to_be_deleted);
            }
            for(auto loop_to_be_deleted: loopsToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(loop_to_be_deleted);
            }
            for(auto face_to_be_deleted: facesToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(face_to_be_deleted);
            }

            // -> 删面的时候别忘了从solids中移除
            for(auto face_to_be_deleted: facesToBeDeleted){
                auto solid = face_to_be_deleted->solid;
                solid->RemoveFace(face_to_be_deleted);
            }

            // 添加的元素   
            MarkNum::GetInstance().AddEntity(v_c);
            for(auto edge_to_be_added: edgesToBeAdded){
                MarkNum::GetInstance().AddEntity(edge_to_be_added);
            }
            for(auto half_edge_to_be_added: halfEdgesToBeAdded){
                MarkNum::GetInstance().AddEntity(half_edge_to_be_added);
            }
            for(auto loop_to_be_added: loopsToBeAdded){
                MarkNum::GetInstance().AddEntity(loop_to_be_added);
            }
            for(auto face_to_be_added: facesToBeAdded){
                MarkNum::GetInstance().AddEntity(face_to_be_added);
            }

            return {e1, e2};
        }

        // 返回值：first是 st~mid 的部分，second是 mid~ed的部分
        std::pair<std::shared_ptr<HalfEdge>, std::shared_ptr<HalfEdge>> SplitHalfEdge(std::shared_ptr<HalfEdge> he, T_NUM param){
            
            param = Utils::clamp(param, 0.0, 1.0);

            if(he == nullptr){
                SPDLOG_ERROR("split half edge is nullptr");
                return {nullptr, nullptr};
            }

            if(he->edge == nullptr){
                SPDLOG_ERROR("split half edge's edge is nullptr");
                return {nullptr, nullptr};
            }

            bool sense = he->sense;

            auto res = SplitEdge(he->edge, (sense)?(1-param):(param));
            if(sense){
                std::swap(res.first, res.second);
            }

            // 这里要做一点处理
            // std::pair<std::shared_ptr<HalfEdge>, std::shared_ptr<HalfEdge>> ans;

            return {res.first->halfEdges.front(), res.second->halfEdges.front()};
        }

        // TODO：应当保证输入的模型在导入的时候排除掉退化边之类的情况
        void CollapseEdge(std::shared_ptr<Edge> e_h){
            
            if(e_h == nullptr){
                SPDLOG_ERROR("collapse edge is nullptr");
                return ;
            }

            auto v_a = e_h->st;
            auto v_b = e_h->ed;

            int v_a_id = MarkNum::GetInstance().GetId(v_a);
            int v_b_id = MarkNum::GetInstance().GetId(v_b); // no actually used

            SPDLOG_DEBUG("collapsing edge: {}", MarkNum::GetInstance().GetId(e_h));
            SPDLOG_DEBUG("v_a_id: {}", v_a_id);
            SPDLOG_DEBUG("v_b_id: {}", v_b_id);


            // 被删除元素列表（稍后会被执行删除操作）
            // 点：v_b
            // 边：e_h, 被合并掉的那些边（注意这里还带有非流形的情况）
            // 半边: i_half_edge, ~pre, ~next
            // 面和环：i_half_edge对应的面和环
            std::vector<std::shared_ptr<Edge>> edgesToBeDeleted;
            std::vector<std::shared_ptr<HalfEdge>> halfEdgesToBeDeleted;
            std::vector<std::shared_ptr<Loop>> loopsToBeDeleted;
            std::vector<std::shared_ptr<Face>> facesToBeDeleted;

            // 1. v_a, v_b合并为v_a
            // 这个步骤依赖于edgesMap, 最后这个edgesMap也需要维护
            // 基于延迟删除map中的元素实现
            auto& edgesMap = MarkNum::GetInstance().edgesMap;
            std::vector<decltype(edgesMap.begin())> its;
            std::vector<std::shared_ptr<Edge>> edgesNeedToChange;
            std::set<std::shared_ptr<Edge>> edgesNeedToSkip;

            // 1.1 找到所有和v_b有关的边
            for (auto it = edgesMap.begin(); it != edgesMap.end(); it++){
                auto st = it->second->st;
                auto ed = it->second->ed;

                int e_in_edgesmap_id = MarkNum::GetInstance().GetId(it->second);

                // [Debug]
                // if(e_in_edgesmap_id == 6834){
                //     SPDLOG_DEBUG("e_in_edgesmap_id: {}", e_in_edgesmap_id);
                // }
                
                if(st == v_b || ed == v_b){
                    its.emplace_back(it);
                    edgesNeedToChange.emplace_back(it->second);
                    SPDLOG_DEBUG("edge marked as need to change: {}", e_in_edgesmap_id);
                }
            }

            // 1.2 从edgesMap里面把所有和v_b有关的边删除掉
            for(auto it: its){
                int e_in_edgesmap_id = MarkNum::GetInstance().GetId(it->second);

                edgesMap.erase(it);
                SPDLOG_DEBUG("edge erase from edgesMap: {}", e_in_edgesmap_id);
            }

            // 2. e_pre, e_next 合并为同一条边, 以及对应关系的修改
            for(auto i_half_edge: e_h->halfEdges){
                auto i_half_edge_next = i_half_edge->next;
                auto i_half_edge_pre = i_half_edge->pre;

                auto e_next = i_half_edge_next->edge;
                auto e_pre = i_half_edge_pre->edge;

                std::vector<std::shared_ptr<HalfEdge>> new_halfedges;
                for(auto i_e_pre_half_edge: e_pre->halfEdges){
                    if(i_e_pre_half_edge != i_half_edge_pre){

                        if(i_half_edge->sense){
                            // -> 顺带更新e_pre上的所有halfedge

                            auto st = i_e_pre_half_edge->GetStart();
                            auto ed = i_e_pre_half_edge->GetEnd();
                            if(ed == e_next->st){ // st == v_b &&
                                i_e_pre_half_edge->sense = true;
                            }
                            else if(st == e_next->ed){ // ed == v_b &&
                                i_e_pre_half_edge->sense = true;
                            }
                            else{
                                i_e_pre_half_edge->sense = false;
                            }

                            i_e_pre_half_edge->edge = e_next;
                        }

                        new_halfedges.emplace_back(i_e_pre_half_edge);
                    }
                }

                for(auto i_e_next_half_edge: e_next->halfEdges){
                    if(i_e_next_half_edge != i_half_edge_next){

                        if(!i_half_edge->sense){
                            // -> 顺带更新e_next上的所有halfedge

                            auto st = i_e_next_half_edge->GetStart();
                            auto ed = i_e_next_half_edge->GetEnd();
                            if(ed == e_pre->st){ // st == v_b &&
                                i_e_next_half_edge->sense = true;
                            }
                            else if(st == e_pre->ed){ // ed == v_b &&
                                i_e_next_half_edge->sense = true;
                            }
                            else{
                                i_e_next_half_edge->sense = false;
                            }

                            i_e_next_half_edge->edge = e_pre;
                        
                        }


                        new_halfedges.emplace_back(i_e_next_half_edge);
                    }
                }

                if(!i_half_edge->sense){
                    e_pre->halfEdges = new_halfedges;
                    e_pre->UpdateHalfEdgesPartner();
                }else{
                    e_next->halfEdges = new_halfedges;
                    e_next->UpdateHalfEdgesPartner();
                }

                // 添加被删除元素
                // 半边
                halfEdgesToBeDeleted.emplace_back(i_half_edge);
                halfEdgesToBeDeleted.emplace_back(i_half_edge_pre);
                halfEdgesToBeDeleted.emplace_back(i_half_edge_next);
                // 边

                if(!i_half_edge->sense){
                    edgesNeedToSkip.insert(e_next);
                    edgesToBeDeleted.emplace_back(e_next);
                }else{
                    edgesNeedToSkip.insert(e_pre);
                    edgesToBeDeleted.emplace_back(e_pre);
                }

                // 环和面
                loopsToBeDeleted.emplace_back(i_half_edge->loop);
                facesToBeDeleted.emplace_back(i_half_edge->loop->face);
            }

            // 1.3 重新将没有被删除的边加回去，并对边的顶点做修改
            edgesNeedToSkip.insert(e_h);
            for(auto e: edgesNeedToChange){
                if(edgesNeedToSkip.count(e)) continue; // 显然要跳过~~当前要收缩的边~~ 所有被删除的边事实上都要跳过

                auto e_id = MarkNum::GetInstance().GetId(e);

                int new_st_id = MarkNum::GetInstance().GetId(e->st);
                int new_ed_id = MarkNum::GetInstance().GetId(e->ed);

                if(e->st == v_b){ // 起始点等于v_b，那么改到v_a去，下同
                    e->st = v_a;
                    new_st_id = v_a_id;
                }
                if(e->ed == v_b){
                    e->ed = v_a;
                    new_ed_id = v_a_id;
                }

                if(new_st_id > new_ed_id){
                    std::swap(new_st_id, new_ed_id);
                }

                // 这里在插入之前应该先检查一下理论上应该就ok了
                // 如果下面的判断条件满足，那么说明当前e其实应该是要和it对应的那个边做合并的
                if(auto it = edgesMap.find({new_st_id, new_ed_id}); it != edgesMap.end()){
                    std::vector<std::shared_ptr<HalfEdge>> new_halfedges;
                    auto e2 = it->second;

                    for(auto i_e_half_edge: e->halfEdges){

                        auto i_e_half_edge_st = i_e_half_edge->GetStart();
                        auto i_e_half_edge_ed = i_e_half_edge->GetEnd();

                        if(i_e_half_edge_ed == e2->st){
                            i_e_half_edge->sense = true;
                        }
                        else if(i_e_half_edge_st == e2->ed){
                            i_e_half_edge->sense = true;
                        }
                        else{
                            i_e_half_edge->sense = false;
                        }

                        i_e_half_edge->edge = e2;
                    }

                    new_halfedges.insert(new_halfedges.end(), e->halfEdges.begin(), e->halfEdges.end());
                    new_halfedges.insert(new_halfedges.end(), e2->halfEdges.begin(), e2->halfEdges.end());

                    e2->halfEdges = new_halfedges;
                    e2->UpdateHalfEdgesPartner();

                    // 这里之后就不再将e插入回去了，而是标记为删除边

                    edgesToBeDeleted.emplace_back(e);
                }
                else{ // 否则说明e这个边就只是和v_b相连并且没有别的边另一侧顶点也和e的另一侧顶点相同，也即收缩后不会出现重叠情况。此时正常修改即可

                    edgesMap[{new_st_id, new_ed_id}] = e; 
                    SPDLOG_DEBUG("change e_id: {} ({} {})", e_id, new_st_id, new_ed_id);
                }

            }

            // 3. 删除元素
            // 这里最好是能先把预先要删除和添加的元素先保存到一些数据结构里面，然后再一次性调用AddEntity或者RemoveEntity（当然目前这个操作是纯删除元素所以应该只会用到RemoveEntity）
            MarkNum::GetInstance().RemoveEntity(v_b);
            MarkNum::GetInstance().RemoveEntity(e_h, false); // 此处要阻止对edgesMap的修改，下同
            for(auto half_edge_to_be_deleted: halfEdgesToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(half_edge_to_be_deleted);
            }
            for(auto edge_to_be_deleted: edgesToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(edge_to_be_deleted, false);
            }
            for(auto loop_to_be_deleted: loopsToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(loop_to_be_deleted);
            }
            for(auto face_to_be_deleted: facesToBeDeleted){
                MarkNum::GetInstance().RemoveEntity(face_to_be_deleted);
            }

            // -> 删面的时候别忘了从solid中移除
            for(auto face_to_be_deleted: facesToBeDeleted){
                auto solid = face_to_be_deleted->solid;
                solid->RemoveFace(face_to_be_deleted);
            }
        }

        void CollapseHalfEdge(std::shared_ptr<HalfEdge> he){
            if(he == nullptr){
                SPDLOG_ERROR("collapse half edge is nullptr");
                return ;
            }

            if(he->edge == nullptr){
                SPDLOG_ERROR("collapse half edge's edge is nullptr");
                return;
            }

            SPDLOG_DEBUG("Collapsing he: {} ({} {})", MarkNum::GetInstance().GetId(he), MarkNum::GetInstance().GetId(he->GetStart()), MarkNum::GetInstance().GetId(he->GetEnd()));
            CollapseEdge(he->edge);
        }

        void CollapseWithTwoVertices(const std::shared_ptr<Vertex>& v_a, const std::shared_ptr<Vertex>& v_b){
            auto e = MarkNum::GetInstance().FindEdgeBetweenVertices(v_a, v_b);
            if(e){
                CollapseEdge(e);
            }
            else{
                // 此处假设v2就是v_b
                int v_a_id = MarkNum::GetInstance().GetInstance().GetId(v_a);
                int v_b_id = MarkNum::GetInstance().GetInstance().GetId(v_b);

                SPDLOG_DEBUG("collapsing with two vertices: v_a_id:{} v_b_id:{}", v_a_id, v_b_id);

                // 被删除元素列表（稍后会被执行删除操作）
                // 点：v_b
                // 边：e_h, 被合并掉的那些边（注意这里还带有非流形的情况）
                auto& edgesMap = MarkNum::GetInstance().edgesMap;
                std::vector<decltype(edgesMap.begin())> its;
                std::vector<std::shared_ptr<Edge>> edgesNeedToChange;

                std::vector<std::shared_ptr<Edge>> edgesToBeDeleted;

                // 1.1 找到所有和v_b有关的边
                for (auto it = edgesMap.begin(); it != edgesMap.end(); it++){
                    auto st = it->second->st;
                    auto ed = it->second->ed;

                    int e_in_edgesmap_id = MarkNum::GetInstance().GetId(it->second);
                    
                    if(st == v_b || ed == v_b){
                        its.emplace_back(it);
                        edgesNeedToChange.emplace_back(it->second);
                        SPDLOG_DEBUG("edge marked as need to change: {}", e_in_edgesmap_id);
                    }
                }
                // 1.2 从edgesMap里面把所有和v_b有关的边删除掉
                for(auto it: its){
                    int e_in_edgesmap_id = MarkNum::GetInstance().GetId(it->second);

                    edgesMap.erase(it);
                    SPDLOG_DEBUG("edge erase from edgesMap: {}", e_in_edgesmap_id);
                }

                // 1.3 重新将没有被删除的边加回去，并对边的顶点做修改
                for(auto e: edgesNeedToChange){
                    auto e_id = MarkNum::GetInstance().GetId(e);

                    int new_st_id = MarkNum::GetInstance().GetId(e->st);
                    int new_ed_id = MarkNum::GetInstance().GetId(e->ed);

                    if(e->st == v_b){ // 起始点等于v_b，那么改到v_a去，下同
                        e->st = v_a;
                        new_st_id = v_a_id;
                    }
                    if(e->ed == v_b){
                        e->ed = v_a;
                        new_ed_id = v_a_id;
                    }

                    if(new_st_id > new_ed_id){
                        std::swap(new_st_id, new_ed_id);
                    }

                     // 如果下面的判断条件满足，那么说明当前e其实应该是要和it对应的那个边做合并的
                    if(auto it = edgesMap.find({new_st_id, new_ed_id}); it != edgesMap.end()){
                        std::vector<std::shared_ptr<HalfEdge>> new_halfedges;
                        auto e2 = it->second;

                        for(auto i_e_half_edge: e->halfEdges){

                            auto i_e_half_edge_st = i_e_half_edge->GetStart();
                            auto i_e_half_edge_ed = i_e_half_edge->GetEnd();

                            if(i_e_half_edge_ed == e2->st){
                                i_e_half_edge->sense = true;
                            }
                            else if(i_e_half_edge_st == e2->ed){
                                i_e_half_edge->sense = true;
                            }
                            else{
                                i_e_half_edge->sense = false;
                            }

                            i_e_half_edge->edge = e2;
                        }

                        new_halfedges.insert(new_halfedges.end(), e->halfEdges.begin(), e->halfEdges.end());
                        new_halfedges.insert(new_halfedges.end(), e2->halfEdges.begin(), e2->halfEdges.end());

                        e2->halfEdges = new_halfedges;
                        e2->UpdateHalfEdgesPartner();

                        // 这里之后就不再将e插入回去了，而是标记为删除边

                        edgesToBeDeleted.emplace_back(e);
                    }
                    else{ // 否则说明e这个边就只是和v_b相连并且没有别的边另一侧顶点也和e的另一侧顶点相同，也即收缩后不会出现重叠情况。此时正常修改即可
                        edgesMap[{new_st_id, new_ed_id}] = e; 
                        SPDLOG_DEBUG("change e_id: {} ({} {})", e_id, new_st_id, new_ed_id);
                    }

                }

                // 3. 删除元素

                MarkNum::GetInstance().RemoveEntity(v_b);
                for(auto edge_to_be_deleted: edgesToBeDeleted){
                    MarkNum::GetInstance().RemoveEntity(edge_to_be_deleted, false);
                }
            }
        }
    
    }

    struct StitchFixer2{
    public:

        struct PoorEdge{
            std::shared_ptr<Edge> edge;
            bool sense; // false: 同向， true：反向

            PoorEdge() : edge(nullptr), sense(false){}

            PoorEdge(const std::shared_ptr<Edge>& e, bool sense): edge(e), sense(sense) {}

            std::shared_ptr<Vertex> GetStart() const {
                return (sense)?(edge->ed):(edge->st);
            }

            std::shared_ptr<Vertex> GetEnd() const {
                return (sense)?(edge->st):(edge->ed);
            }

            T_NUM Length() const{
                return edge->Length();
            }

            std::pair<PoorEdge, PoorEdge> Split(T_NUM param){
                param = Utils::clamp(param, 0.0, 1.0);

                if(edge == nullptr){
                    SPDLOG_ERROR("The edge of poor edge is nullptr.");
                    return {PoorEdge(), PoorEdge()};
                }

                bool sense = this->sense;

                auto res = GeometryUtils::SplitEdge(edge, (sense)?(1-param):(param));
                if(sense){
                    std::swap(res.first, res.second);
                }

                return {PoorEdge(res.first, sense), PoorEdge(res.second, sense)};
            }
        };

        std::shared_ptr<Solid> solid_ptr;
        std::vector<std::shared_ptr<Edge>> poorEdges;

        using RingType = std::vector<PoorEdge>;
        std::vector<RingType> rings;

        Coordinate lower_bound_coord{MAXVAL, MAXVAL, MAXVAL}, upper_bound_coord{MINVAL, MINVAL, MINVAL};

        StitchFixer2(const std::shared_ptr<Solid>& solid): solid_ptr(solid){}

        bool Start(bool call_fix){
            SPDLOG_INFO("Start.");

            Clear();

            CollapseSameVertices();
            CalculateBoundingBox();
            FindPoorEdges();
            FindRings();
            bool res = CheckRingsBox();
            if(res)
            {
                DoRings();
            }
            else{
                SPDLOG_INFO("End.");
                return false;
            }

            SPDLOG_INFO("End.");
            return true;
        }

        void Clear(){
            poorEdges.clear();
            rings.clear();
            lower_bound_coord = Coordinate{MAXVAL, MAXVAL, MAXVAL};
            upper_bound_coord = Coordinate{MINVAL, MINVAL, MINVAL};
        }

        void Status(){
            // TODO
        }

        void Test(){
            SPDLOG_INFO("start.");

            // 先临时弄成循环遍历那样？
            int poor_coedge_count[11];
            memset(poor_coedge_count, 0, sizeof(poor_coedge_count));

            for(auto f: solid_ptr->faces){
                auto lp = f->st;

                auto i_half_edge = lp->st;

                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_ERROR("i_half_edge is null");
                        break;
                    }

                    // 此处已经遍历到了所有halfedges
                    int partner_count = GeometryUtils::EdgePartnerCount(i_half_edge->edge);
                    if(partner_count >= 1){
                        // [有效性检查]
                        if(i_half_edge->GetStart() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO START: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }
                        if(i_half_edge->GetEnd() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO END: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }
                        // [有效性检查] END
                        
                        // 贡献
                        if(partner_count <= 10){
                            poor_coedge_count[partner_count]++;
                        }
                        else{
                            poor_coedge_count[0]++;
                        }

                    }

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);

            }

            for(int i=0;i<=10;i++){
                SPDLOG_INFO("test poor edge total num for nonmanifold {}: {}", i, poor_coedge_count[i]);
            }


            SPDLOG_INFO("end.");
        }

    private:

        void CollapseSameVertices(){
            SPDLOG_INFO("start.");

            std::set<std::shared_ptr<Edge>> degeneratedEdges;

            do{
                degeneratedEdges.clear();

                for(auto f: solid_ptr->faces){
                    auto lp = f->st;
                    auto i_half_edge = lp->st;

                    do{
                        if(i_half_edge == nullptr){
                            SPDLOG_ERROR("i_half_edge is null");
                            break;
                        }

                        auto i_edge = i_half_edge->edge;
                        if(i_edge != nullptr && i_edge->st->pointCoord == i_edge->ed->pointCoord){ // 首尾相同
                            SPDLOG_DEBUG("same vertices for edge: {}, half_edge: {}, st: {} ({} {} {}), ed: {} ({} {} {})",
                                MarkNum::GetInstance().GetId(i_edge),
                                MarkNum::GetInstance().GetId(i_half_edge),
                                MarkNum::GetInstance().GetId(i_edge->st),
                                i_edge->st->pointCoord.x(),
                                i_edge->st->pointCoord.y(),
                                i_edge->st->pointCoord.z(),
                                MarkNum::GetInstance().GetId(i_edge->ed),
                                i_edge->ed->pointCoord.x(),
                                i_edge->ed->pointCoord.y(),
                                i_edge->ed->pointCoord.z()
                            );

                            degeneratedEdges.insert(i_edge);
                        }

                        i_half_edge = i_half_edge->next;
                    }while(i_half_edge && i_half_edge != lp->st);
                }

                SPDLOG_DEBUG("degeneratedEdges size: {}", degeneratedEdges.size());
                
                for(auto de: degeneratedEdges){
                    SPDLOG_DEBUG("degenerated edge will be collapsed: {}", MarkNum::GetInstance().GetId(de));
                    GeometryUtils::CollapseEdge(de);
                }

            }while(degeneratedEdges.size());

            SPDLOG_INFO("end.");
        }

        void CalculateBoundingBox(){
            SPDLOG_INFO("start.");

            lower_bound_coord = Coordinate{MAXVAL, MAXVAL, MAXVAL};
            upper_bound_coord = Coordinate{MINVAL, MINVAL, MINVAL};

            for(auto f: solid_ptr->faces){
                auto lp = f->st;
                auto i_half_edge = lp->st;

                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_ERROR("i_half_edge is null");
                        break;
                    }

                    auto st = i_half_edge->GetStart();
                    auto ed = i_half_edge->GetEnd();

                    lower_bound_coord = lower_bound_coord.Min(st->pointCoord);
                    lower_bound_coord = lower_bound_coord.Min(ed->pointCoord);

                    upper_bound_coord = upper_bound_coord.Max(st->pointCoord);
                    upper_bound_coord = upper_bound_coord.Max(ed->pointCoord);
                    
                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);
            }

            SPDLOG_INFO("end.");
        }

        std::pair<Coordinate, Coordinate> CalculateRingBox(const RingType& ring){
            std::pair<Coordinate, Coordinate> ans{{MAXVAL, MAXVAL, MAXVAL}, {MINVAL, MINVAL, MINVAL}};
            
            for(auto& pe: ring){
                auto st = pe.GetStart();
                auto ed = pe.GetEnd();

                ans.first = ans.first.Min(st->pointCoord);
                ans.first = ans.first.Min(ed->pointCoord);

                ans.second = ans.second.Max(st->pointCoord);
                ans.second = ans.second.Max(ed->pointCoord);
            }

            return ans;
        }

        bool CheckRingsBox(){
            SPDLOG_INFO("start.");

            // 模型的包围盒体积
            T_NUM solid_box_volume = lower_bound_coord.Volume(upper_bound_coord);

            SPDLOG_DEBUG("solid_box_volume: {}", solid_box_volume);

            for(auto& ring: rings){
                auto box_point_pair = CalculateRingBox(ring);
                T_NUM ring_box_volume = box_point_pair.first.Volume(box_point_pair.second);

                SPDLOG_DEBUG("ring_box_volume / solid_box_volume > {}: ring size: {}, ring_box_volume: {}", VOLUME_PROPORTION_THRESHOLD, ring.size(),  ring_box_volume);

                if(ring_box_volume / solid_box_volume > VOLUME_PROPORTION_THRESHOLD){ // 这个值合适吗
                    SPDLOG_DEBUG("ring_box_volume / solid_box_volume > {}: ring size: {}", ring.size(), VOLUME_PROPORTION_THRESHOLD);
                    SPDLOG_INFO("end.");
                    return false;
                }
            }


            SPDLOG_INFO("end.");

            return true;
        }

        /*
            调用顺序：1
            找破边，并保存到poor_coedge_vec中
        */
        void FindPoorEdges(){
            SPDLOG_INFO("start.");

            // 先临时弄成循环遍历那样？

            for(auto f: solid_ptr->faces){
                auto lp = f->st;

                auto i_half_edge = lp->st;

                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_ERROR("i_half_edge is null");
                        break;
                    }

                    // 此处已经遍历到了所有halfedges
                    int partner_count = GeometryUtils::EdgePartnerCount(i_half_edge->edge);

                    // 版本1：当Edge的count为奇数时，满足作为poorEdge的条件
                    // 版本2：当Edge的count为1或者3以上的非流形边时（偶数也算），满足作为poorEdge的条件
                    // 目前以版本1为这里的实现
                    if(partner_count % 2){
                        // [有效性检查]
                        if(i_half_edge->GetStart() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO START: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }
                        if(i_half_edge->GetEnd() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO END: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }
                        // [有效性检查] END
                        
                        // 构造poor_coedge
                        poorEdges.emplace_back(i_half_edge->edge);

                    }
                    
                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);
            }

            SPDLOG_INFO("poor edges total num: {}", static_cast<int>(poorEdges.size()));
            SPDLOG_INFO("end.");
        }

        /*
            调用顺序：2
            找环，并保存到rings中
            现在改用找点双分量的方法
        */
        void FindRings(){
            SPDLOG_INFO("start.");

            std::map<int, std::vector<std::shared_ptr<Edge>>> vdcc_to_poor_edges; // tarjan部分的输出：保存对应vdcc中的边（顺序不一定，如果vdcc是孤立点也可能为空）; vdcc_id -> edges

            // 确定无向图点双连通分量，并且确定各个点双连通分量里面是否是构成环
            {
                struct TarjanNode{
                    int dfn, low; // dfn, low: dfs遍历序以及最小可追溯值
                    int c; // c: 一般点所属的edcc编号
                    bool is_cut; // is_cut: 是否为割点
                };

                struct TarjanEdge{
                    int to_vertex_id; // to: 顶点id
                    int inext; // 临时下标
                    std::shared_ptr<Edge> original_edge; // 这个实际上没用
                };

                std::map<int, TarjanNode> tarjan_nodes;
                std::vector<TarjanEdge> tarjan_edges;
                std::map<int, int> head; // node_id -> start tarjan_edge

                int dfn = 0;
                std::vector<std::vector<int>> vdccs;
                std::map<int, std::set<int>> vertex_to_vdcc; // 在求得vdccs之后，通过变换得到，用来求边在哪个vdcc里面

                std::vector<int> st; // 保存顶点id的栈

                auto add_tarjan_edge = [&] (const std::shared_ptr<Edge>& original_edge){
                    int from_vertex_id = MarkNum::GetInstance().GetId(original_edge->st);
                    int to_vertex_id = MarkNum::GetInstance().GetId(original_edge->ed);

                    if(tarjan_nodes.count(from_vertex_id) == 0){
                        TarjanNode tn;
                        tn.dfn = tn.low = 0;
                        tn.c = -1;
                        tn.is_cut = false;

                        tarjan_nodes[from_vertex_id] = tn;
                    }

                    if(tarjan_nodes.count(to_vertex_id) == 0){
                        TarjanNode tn;
                        tn.dfn = tn.low = 0;
                        tn.c = -1;
                        tn.is_cut = false;

                        tarjan_nodes[to_vertex_id] = tn;
                    }

                    // 正
                    TarjanEdge te;
                    te.to_vertex_id = to_vertex_id;
                    if(head.count(from_vertex_id)){
                        te.inext = head[from_vertex_id];
                    }
                    else{
                        te.inext = -1;
                    }
                    te.original_edge = original_edge;

                    head[from_vertex_id] = tarjan_edges.size();
                    tarjan_edges.emplace_back(te);

                    // 反
                    TarjanEdge te2;
                    te2.to_vertex_id = from_vertex_id;
                    if(head.count(to_vertex_id)){
                        te2.inext = head[to_vertex_id];
                    }
                    else{
                        te2.inext = -1;
                    }
                    te2.original_edge = original_edge;

                    head[to_vertex_id] = tarjan_edges.size();
                    tarjan_edges.emplace_back(te2);

                };

                std::function<void(int, int)> tarjan = [&] (int x_vertex_id, int root_vertex_id) -> void {
                    tarjan_nodes[x_vertex_id].dfn = tarjan_nodes[x_vertex_id].low = ++dfn;
                    if(x_vertex_id == root_vertex_id && head.count(x_vertex_id) == 0){ // 孤立点
                        vdccs.emplace_back();
                        vdccs.back().emplace_back(x_vertex_id);

                        return ;
                    }

                    st.emplace_back(x_vertex_id);

                    int flag = 0;
                    for(int e = head[x_vertex_id]; e!=-1; e=tarjan_edges[e].inext){

                        int y = tarjan_edges[e].to_vertex_id;
                        if(!tarjan_nodes[y].dfn){
                            tarjan(y, root_vertex_id);
                            tarjan_nodes[x_vertex_id].low = std::min(tarjan_nodes[x_vertex_id].low, tarjan_nodes[y].low);

                            if(tarjan_nodes[x_vertex_id].dfn <= tarjan_nodes[y].low){
                                flag++;
                                if(x_vertex_id != root_vertex_id || flag >1){
                                    tarjan_nodes[x_vertex_id].is_cut = true;
                                }

                                vdccs.emplace_back();
                                int z;
                                do{
                                    z = st.back();
                                    st.pop_back();
                                    vdccs.back().emplace_back(z);
                                } while(z!=y);
                                vdccs.back().emplace_back(x_vertex_id);
                            }
                        }
                        else{
                            tarjan_nodes[x_vertex_id].low = std::min(tarjan_nodes[x_vertex_id].low, tarjan_nodes[y].dfn);
                        }
                    }
                };

                // 添加用来执行tarjan算法的边
                for(auto pe: poorEdges){
                    add_tarjan_edge(pe);
                }

                // 对每个没被遍历过的点执行tarjan，找出点双
                for(auto tn_pair: tarjan_nodes){
                    if(tn_pair.second.dfn==0){
                        tarjan(tn_pair.first, tn_pair.first);
                    }
                }

                // 通过vdccs构造vertex_to_vdcc
                for(int i=0;i<vdccs.size();i++){
                    auto& vs = vdccs[i];
                    for(auto v: vs){
                        vertex_to_vdcc[v].insert(i);
                    }
                }

                // 遍历每个边，取得首尾顶点的对应vdcc集合，将集合求交后获得交集，检查交集中是否应该只有一个元素，如果是的话那么那个元素就是对应边所属vdcc
                for(auto pe: poorEdges){
                    int st_id = MarkNum::GetInstance().GetId(pe->st);
                    int ed_id = MarkNum::GetInstance().GetId(pe->ed);

                    auto st_vdccs = vertex_to_vdcc[st_id];
                    auto ed_vdccs = vertex_to_vdcc[ed_id];

                    std::set<int> intersected_vdccs;
                    set_intersection(st_vdccs.begin(), st_vdccs.end(), ed_vdccs.begin(), ed_vdccs.end(), std::inserter(intersected_vdccs, intersected_vdccs.begin()));

                    if(intersected_vdccs.size() == 1){
                        auto vdcc_id = *intersected_vdccs.begin();
                        vdcc_to_poor_edges[vdcc_id].emplace_back(pe);
                    }
                    else{
                        SPDLOG_ERROR("size of intersected_vdccs not equal to 1: size: {}, pe: {}", intersected_vdccs.size(), MarkNum::GetInstance().GetId(pe));
                    }
                }
            }


            // 取得强连通分量中的边集合，然后检查边是否满足串成一个环
            for(auto vdcc2edges_pair: vdcc_to_poor_edges){
                int vdcc_id = vdcc2edges_pair.first;
                auto edges = vdcc2edges_pair.second;
                
                SPDLOG_DEBUG("edges size of vdcc {}: {}", vdcc_id, edges.size());
                if(edges.size() >=2){
                    // 稍等，这里需要另一个类来包装一下poor edge的成环方向

                    std::map<std::shared_ptr<Vertex>, std::set<std::shared_ptr<Edge>>> v_to_edges;

                    for(auto pe: edges){
                        v_to_edges[pe->st].insert(pe);
                        v_to_edges[pe->ed].insert(pe);
                    }

                    // 判断是否满足成环条件：每个顶点相邻边都为2个
                    bool flag = true;
                    for(auto& ve_pair: v_to_edges){
                        if(ve_pair.second.size() != 2){
                            flag = false;
                            SPDLOG_DEBUG("This vdcc {} is not a loop: size of adjacent edges of the vertex {} is: {}", vdcc_id, MarkNum::GetInstance().GetId(ve_pair.first), ve_pair.second.size());
                        }
                    }

                    // 若满足条件，则继续将环上边弄成一个ring
                    if(flag){
                        SPDLOG_DEBUG("This vdcc {} is a loop.", vdcc_id);

                        // 这里就比较有意思了，可能要先去找一个单面边（红边），以其为基准作为初始的内容
                        // 目前先粗暴一点，如果没有红边就不管了
                        std::shared_ptr<Edge> candidate_poor_edge = nullptr;
                        for(auto pe: edges){
                            if(GeometryUtils::EdgePartnerCount(pe) == 1){
                                candidate_poor_edge = pe;
                            }
                        }

                        if(candidate_poor_edge){
                            // 有红边的情况，以这个candidate_poor_edge的唯一的half edge为方向，然后找到环上各个边，同样标记方向
                            std::set<std::shared_ptr<Edge>> visited_edges;

                            RingType ring;

                            // 先把candidate_poor_edges塞进去，sense采用当中唯一的half edge的方向
                            visited_edges.insert(candidate_poor_edge);
                            PoorEdge candidate_pe{candidate_poor_edge, candidate_poor_edge->halfEdges.front()->sense};
                            ring.emplace_back(candidate_pe);

                            // 利用v_to_edges遍历环以获取顺序
                            for(auto v = candidate_pe.GetEnd();;){
                                bool flag = false;
                                for(auto next_e: v_to_edges[v]){
                                    if(visited_edges.count(next_e)){
                                        continue;
                                    }
                                    else{
                                        candidate_pe.edge = next_e;
                                        candidate_pe.sense = (next_e->st == v)?(false):(true);
                                        ring.emplace_back(candidate_pe);

                                        visited_edges.insert(next_e);

                                        v = candidate_pe.GetEnd();
                                        flag = true;
                                        break;
                                    }
                                }

                                if(flag == false){
                                    break;
                                }
                            }

                            SPDLOG_DEBUG("The size of the loop of vdcc {}: {}", vdcc_id, ring.size());
                            rings.emplace_back(ring);
                        }
                        else{
                            SPDLOG_DEBUG("The loop of vdcc {} has not candidate poor edge.", vdcc_id);
                        }
                        
                    }
                }
                else{
                    SPDLOG_DEBUG("edges size of vdcc {} is less than 2.", vdcc_id);
                }
            }

            // 打印rings中已经找到的环的信息
            SPDLOG_DEBUG("rings size: {}", rings.size());

            for(auto ring: rings){
                SPDLOG_DEBUG("found ring size: {}", ring.size());
                for(auto poor_edge: ring){
                    SPDLOG_DEBUG("poor_edge in ring: edge id: {} (st: {} ({}, {}, {}), ed: {} ({}, {}, {}), sense: {})", 
                        MarkNum::GetInstance().GetId(poor_edge.edge), 
                        MarkNum::GetInstance().GetId(poor_edge.GetStart()), 
                        (poor_edge.GetStart()->pointCoord.x()), 
                        (poor_edge.GetStart()->pointCoord.y()), 
                        (poor_edge.GetStart()->pointCoord.z()), 
                        MarkNum::GetInstance().GetId(poor_edge.GetEnd()), 
                        (poor_edge.GetEnd()->pointCoord.x()), 
                        (poor_edge.GetEnd()->pointCoord.y()), 
                        (poor_edge.GetEnd()->pointCoord.z()),
                        (poor_edge.sense)
                    );
                }
            }

            SPDLOG_INFO("end.");
        }

        void DoRings(){
            struct ListNode{
                PoorEdge poorEdge;
                T_NUM st_param;
                T_NUM ed_param;

                ListNode() {}
                ListNode(const PoorEdge& pe, T_NUM st_param, T_NUM ed_param): poorEdge(pe), st_param(st_param), ed_param(ed_param){}
            };

            using ListType = std::list<ListNode>;
            using VertexMapType = std::map<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>;

            auto debug_list = [&] (const ListType& part){
            
                SPDLOG_DEBUG("list size: {}", part.size());
                for(auto node: part){
                
                    SPDLOG_DEBUG("edge: {}, sense: {}, st: {}, ed: {}, st_param: {}, ed_param: {}", 
                        MarkNum::GetInstance().GetId(node.poorEdge.edge),
                        node.poorEdge.sense,
                        MarkNum::GetInstance().GetId(node.poorEdge.GetStart()),
                        MarkNum::GetInstance().GetId(node.poorEdge.GetEnd()),
                        node.st_param,
                        node.ed_param
                    );
                }
            };

            auto debug_map = [&] (const VertexMapType& m){
                SPDLOG_DEBUG("map size: {}", m.size());
                for(auto p: m){
                    SPDLOG_DEBUG("map vertices pair: {} {}", MarkNum::GetInstance().GetId(p.first), MarkNum::GetInstance().GetId(p.second));
                }
            };

            auto get_split_ring_indices_by_distance = [&] (const RingType& ring, int& max_index_i, int& max_index_j){
                T_NUM max_dis = MINVAL;
                max_index_i = -1;
                max_index_j = -1;

                for(int i=0;i<ring.size();i++){
                    for(int j=i+1;j<ring.size();j++){
                        auto c1 = ring[i].GetStart()->pointCoord;
                        auto c2 = ring[j].GetStart()->pointCoord;
                        auto dis = c1.Distance(c2);

                        if(dis > max_dis){
                            max_index_i = i;
                            max_index_j = j;
                            max_dis = dis;
                        }
                    }
                }
            };

            auto get_split_ring_indices_by_balance = [&] (const RingType& ring, int& max_index_i, int& max_index_j){
                T_NUM dis_sum = 0.0;

                for(int i=0;i<ring.size();i++){
                    dis_sum += ring[i].Length();
                }

                std::vector<T_NUM> proportions;

                {
                    T_NUM temp_sum = 0.0;
                    for(int i=0; i<ring.size(); i++){
                        proportions.emplace_back(temp_sum / dis_sum);
                        temp_sum += ring[i].Length();
                    }
                }

                max_index_i = -1;
                max_index_j = -1;
                T_NUM temp_proportion = 2.0;

                for(int i=0;i<ring.size();i++){

                    if(proportions[i] > 0.5) {
                        break;
                    }
                    
                    for(int j=i+1;j<ring.size();j++){
                        auto new_proportion = std::abs(proportions[j]-proportions[i]-0.5);
                        if(new_proportion < temp_proportion){
                            temp_proportion = new_proportion;
                            max_index_i = i;
                            max_index_j = j;
                        }
                        
                    }
                }
            
            };

            auto split_ring = [&] (RingType& ring, ListType& part, int max_index_i, int max_index_j, bool reverse_param){
                T_NUM part_dis_sum = 0.0;
                for(int i=max_index_i; i!=max_index_j; i=(i+1)%ring.size()){
                    part.emplace_back(ring[i], 0.0, 0.0);
                    part_dis_sum += ring[i].Length();
                }

                T_NUM temp_dis_sum = 0.0;
                for(auto& list_node: part){
                    list_node.st_param = (reverse_param)?(1.0-temp_dis_sum):(temp_dis_sum);
                    temp_dis_sum += (list_node.poorEdge.Length()) / part_dis_sum;
                    list_node.ed_param = (reverse_param)?(1.0-temp_dis_sum):(temp_dis_sum);
                }
            };

            auto match_vertices = [&] (const ListType& part1, const ListType& part2, VertexMapType& part1_vmap, VertexMapType& part2_vmap){
                // first pass
                auto rit = part2.rbegin();
                for(auto it = part1.begin(); it != part1.end(); it++){
                    if(it == part1.begin()){
                        continue;
                    }

                    auto pe = it->poorEdge;
                    auto param = it->st_param;

                    auto most_right_rit = part2.rend();

                    while(rit!=part2.rend() && (std::next(rit)!=part2.rend() && rit->st_param <= param)){
                        if(param - rit->st_param <= 0.5){
                            most_right_rit = rit;
                        }
                        rit++;
                    }

                    if(most_right_rit != part2.rend()){
                        auto v1 = pe.GetStart();
                        auto v2 = most_right_rit->poorEdge.GetStart();

                        part1_vmap[v1] = v2;
                        part2_vmap[v2] = v1;
                    }
                }

                // second pass
                auto it = part1.begin();
                if(it != part1.end()){
                    it++;
                }

                for(auto rit = part2.rbegin(); rit != part2.rend(); rit++){
                    if(std::next(rit) == part2.rend()){
                        continue;
                    }

                    auto pe = rit->poorEdge;
                    auto param = rit->st_param;

                    if(part2_vmap.count(pe.GetStart())){
                        continue;
                    }

                    auto most_right_it = part1.end();

                    while(it != part1.end() && it->st_param <= param){
                        if(param - it->st_param <= PARAM_EPSLION && (part1_vmap.count(it->poorEdge.GetStart()) == 0)){ // 这里要额外：已经配对过的顶点要跳过
                            most_right_it = it;
                        }
                        it++;
                    }

                    if(most_right_it != part1.end()){
                    
                        auto v1 = most_right_it->poorEdge.GetStart();
                        auto v2 = pe.GetStart();
                        part1_vmap[v1] = v2;
                        part2_vmap[v2] = v1;
                    }
                }
            };

            // part1: 分割点参数提供者
            // part2: 被分割的那部分
            // 这是一个n^2算法
            // 注意这个函数的写法是不对称的, 也即要分成两个pass
            auto split_part = [&] (ListType& part1, ListType& part2, VertexMapType& part1_vmap, VertexMapType& part2_vmap){
                // first pass
                for(auto it = part1.begin(); it != part1.end(); it++){
                    if(it == part1.begin()){ // 这么写万一是空容器也可应对
                        continue;
                    }

                    auto pe = it->poorEdge;
                    auto param = it->st_param;

                    if(part1_vmap.count(pe.GetStart()) == 0){// 未有匹配，则需要在对面分割
                        for(auto rit = part2.rbegin(); rit != part2.rend(); rit++){
                            if(rit->st_param > param && rit->ed_param < param){ // 拆分
                                auto pe_need_to_split = rit->poorEdge;
                                auto new_param = (rit->st_param - param) / (rit->st_param - rit->ed_param);

                                auto poor_edges_split_pair = pe_need_to_split.Split(new_param);
                                auto new_vertex = poor_edges_split_pair.first.GetEnd();

                                // 第一个新分裂出的半边
                                ListNode node1{
                                    poor_edges_split_pair.first,
                                    rit->st_param,
                                    param
                                };
                                
                                // 第二个新分裂出的半边
                                ListNode node2{
                                    poor_edges_split_pair.second,
                                    param,
                                    rit->ed_param
                                };

                                // 删除
                                auto rit_base = rit.base();
                                --rit_base;
                                auto insert_pos = part2.erase(rit_base);

                                // 插入
                                part2.insert(insert_pos, node1);
                                part2.insert(insert_pos, node2);

                                // 令拆分后新生成的点与当前未有匹配的那个点匹配
                                part1_vmap[pe.GetStart()] = new_vertex;
                                part2_vmap[new_vertex] = pe.GetStart();

                                break;
                            }
                        }
                    }
                }
            
                // second pass
                for(auto rit = part2.rbegin(); rit != part2.rend(); rit++){
                    if(std::next(rit) == part2.rend()){
                        continue;
                    }

                    auto pe = rit->poorEdge;
                    auto param = rit->st_param;

                    if(part2_vmap.count(pe.GetStart()) == 0){// 没有匹配
                        for(auto it = part1.begin(); it != part1.end(); it++){
                            if(it->st_param < param && it->ed_param > param){
                                auto pe_need_to_split = it->poorEdge;
                                auto new_param = (param - it->st_param) / (it->ed_param - it->st_param);

                                auto poor_edges_split_pair = pe_need_to_split.Split(new_param);

                                auto new_vertex = poor_edges_split_pair.first.GetEnd();

                                // 第一个新分裂出的半边
                                ListNode node1{
                                    poor_edges_split_pair.first,
                                    it->st_param,
                                    param
                                };
                                
                                // 第二个新分裂出的半边
                                ListNode node2{
                                    poor_edges_split_pair.second,
                                    param,
                                    it->ed_param
                                };

                                // 删除
                                auto insert_pos = part1.erase(it);

                                // 插入
                                part1.insert(insert_pos, node1);
                                part1.insert(insert_pos, node2);
                                
                                // 令拆分后新生成的点与当前未有匹配的那个点匹配
                                part2_vmap[pe.GetStart()] = new_vertex;
                                part1_vmap[new_vertex] = pe.GetStart();

                                break;
                            }
                        }
                    }
                }
            };

            // 注意：这里顺序会有很大影响！
            auto merge_match_vertices = [&] (const ListType& part1, const VertexMapType& part1_vmap, const VertexMapType& part2_vmap){
                int n = 0;

                for(auto it = part1.begin(); it != part1.end();it++){
                    if(it == part1.begin()){
                        continue;
                    }

                    auto pe = it->poorEdge;
                    auto st = pe.GetStart();

                    if(auto mit = part1_vmap.find(st); mit != part1_vmap.end()){
                        auto match_vertex = mit->second;
                        GeometryUtils::CollapseWithTwoVertices(st, match_vertex);
                        SPDLOG_DEBUG("merge done. n: {}", n);
                    }
                    else{
                        SPDLOG_ERROR("match vertex not found in merging match vertices: st: {}, n: {}", MarkNum::GetInstance().GetId(st), n);
                    }

                    n++;

                    // if(n>=1){
                    //     break;
                    // }
                }
            };

            int rn = 0;
            for(auto& ring: rings){
                if(ring.size()<2){
                    SPDLOG_ERROR("ring size is less than 2.");
                    break;
                }
                else{
                    SPDLOG_DEBUG("processing ring size: {}", ring.size());
                }

                // 1. 分割环
                // 此处改成：找到两个点，使得这两个点两侧的长度尽可能相同
                int max_index_i = -1;
                int max_index_j = -1;
                // get_split_ring_indices_by_distance(ring, max_index_i, max_index_j);
                get_split_ring_indices_by_balance(ring, max_index_i, max_index_j);

                if(max_index_i == -1 || max_index_j == -1){
                    SPDLOG_ERROR("No found for max_index_i or max_index_j");
                    break;
                }

                // -> 把分割后的分成两部分（方便后续遍历），并更新他们的st参数值
                // 这里都顺着存吧，一会找对面的时候记得用1减去对面的参数来配对就好

                ListType part1, part2;
                split_ring(ring, part1, max_index_i, max_index_j, false);
                debug_list(part1);
                split_ring(ring, part2, max_index_j, max_index_i, true);
                debug_list(part2);

                // -> (可能没必要)根据线段数多寡交换，使得：part1始终是较少的那个，part2始终是较多的那个

                // 2. 配对
                // 匹配的结果保存在一个map<vertex*, vertex*>里面
                VertexMapType part1_vmap, part2_vmap;
                match_vertices(part1, part2, part1_vmap, part2_vmap);
                debug_map(part1_vmap);
                debug_map(part2_vmap);

                // 3. 分割多侧并配对
                // 4. 分割少侧并配对
                split_part(part1, part2, part1_vmap, part2_vmap);

                debug_list(part1);
                debug_map(part1_vmap);
                debug_list(part2);
                debug_map(part2_vmap);

                // 5. 配对点缝合
                merge_match_vertices(part1, part1_vmap, part2_vmap);

                // if((++rn)>=4) break;
            }
        }

        void SplitTest(){
            for(auto e: poorEdges){
                GeometryUtils::SplitEdge(e, 0.5);
            }
        }

        void CollapseTest(){
            int n = 0;
            for(auto e: poorEdges){
                if(true){
                    if(auto e_it = MarkNum::GetInstance().markNumMap.find(e.get()); e_it != MarkNum::GetInstance().markNumMap.end())
                    {
                        // SPDLOG_DEBUG("Collapsing he: {} ({} {})", he_it->second.second, MarkNum::GetInstance().markNumMap[he->GetStart().get()].second, MarkNum::GetInstance().markNumMap[he->GetEnd().get()].second);
                        GeometryUtils::CollapseEdge(e);
                        MarkNum::GetInstance().Test();
                    }
                    else{
                        SPDLOG_ERROR("Collapsing e is not exist in markNumMap.");
                    }
                }
                else{
                    break;
                }
                n++;
            }
        }

    };

}
