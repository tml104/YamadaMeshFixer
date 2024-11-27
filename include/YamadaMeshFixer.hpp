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
    const T_NUM GLOBAL_TOLERANCE = 1e-6;

    const T_NUM EPSLION1 = 0.05;
    const T_NUM EPSLION2 = cos(M_PI / 6);

    const T_NUM EPSLION2_SIN = sqrt(1.0 - EPSLION2 * EPSLION2);

  	const T_NUM MINVAL = -1e9;
	const T_NUM MAXVAL = 1e9;

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

        void AddHalfEdge(const std::shared_ptr<HalfEdge>& he){
            halfEdges.emplace_back(he);
        }

        void UpdateHalfEdgesPartner(){
            // 按照halfedge添加顺序串成一个环
            for(int h=0;h<halfEdges.size();h++){
                halfEdges[h]->partner = halfEdges[(h+1) % halfEdges.size()];
            }
        }
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
        std::vector<std::shared_ptr<Face>> faces;

        void AddFace(const std::shared_ptr<Face>& f){
            faces.emplace_back(f);
        }

        void RemoveFace(const std::shared_ptr<Face>& f){
            // TODO: 可能有性能瓶颈，有待改进
            for(auto it = faces.begin();it!=faces.end();it++){
                if(*(it) == f){
                    faces.erase(it);
                    break;
                }
            }
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
    
        void SplitEdge(std::shared_ptr<Edge> e, double param){
            if(e == nullptr){
                SPDLOG_ERROR("split edge is nullptr");
                return ;
            }


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

                // 2.2 HE1, HE2（连接关系稍后维护）
                auto he1 = std::make_shared<HalfEdge>();
                auto he2 = std::make_shared<HalfEdge>();

                // -> HE1, HE2的部分内容设置
                he1->edge = e1;
                he2->edge = e2;
                he1->sense = i_half_edge->sense;
                he2->sense = i_half_edge->sense;

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
            }

            e1->UpdateHalfEdgesPartner();
            e2->UpdateHalfEdgesPartner();
            

            // TODO: 新加与修改元素在MarkNum中的维护
        }

        void SplitHalfEdge(std::shared_ptr<HalfEdge> he, double param){

            if(he == nullptr){
                SPDLOG_ERROR("split half edge is nullptr");
                return ;
            }

            if(he->edge == nullptr){
                SPDLOG_ERROR("split half edge's edge is nullptr");
                return;
            }

            bool sense = he->sense;

            SplitEdge(he->edge, (sense)?(1-param):(param));
            
        }
    
    }

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

    }

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
        std::map<TopoType, std::list<int>> deletedIdLists;

        // 有特殊用处需要维护的数据结构
        std::map<std::pair<int, int>, std::shared_ptr<Edge>> edgesMap; // (vertex id, vertex id) -> edge
        
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

            auto make_solid = [&](const std::vector<std::shared_ptr<Face>>& faces) -> std::shared_ptr<Solid>{

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

                std::vector<std::shared_ptr<Face>> faces;
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
                    faces.emplace_back(face);
                }

                // solid
                auto solid = make_solid(faces);
                solids.emplace_back(solid);

                s++;
            }

            SPDLOG_INFO("End.");
        }

        std::shared_ptr<Edge> FindEdgeBetweenVertices(const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2){
            auto v1_v2_edge_it = edgesMap.find({markNumMap[v1.get()].second, markNumMap[v2.get()].second});

            if(v1_v2_edge_it != edgesMap.end()){
                return v1_v2_edge_it->second;
            }

            auto v2_v1_edge_it = edgesMap.find({markNumMap[v2.get()].second, markNumMap[v1.get()].second});
            if(v2_v1_edge_it != edgesMap.end()){
                return v2_v1_edge_it->second;
            }
            
            return nullptr;
        }

        // 测试
        void Test(){
            SPDLOG_INFO("Start.");

            SPDLOG_INFO("Entity Count: {}", capacities[TopoType::Entity]);
            SPDLOG_INFO("Solid Count: {}", capacities[TopoType::Solid]);
            SPDLOG_INFO("Face Count: {}", capacities[TopoType::Face]);
            SPDLOG_INFO("Loop Count: {}", capacities[TopoType::Loop]);
            SPDLOG_INFO("HalfEdge Count: {}", capacities[TopoType::HalfEdge]);
            SPDLOG_INFO("Edge Count: {}", capacities[TopoType::Edge]);
            SPDLOG_INFO("Vertex Count: {}", capacities[TopoType::Vertex]);

            SPDLOG_INFO("End.");
        }

        // TODO: ...
        void AddEntity(const std::shared_ptr<Entity>& new_entity){
        }

        // TODO: ...
        void RemoveEntity(const std::shared_ptr<Entity>& removed_entity){
        
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

            auto process_vertex = [&](const std::shared_ptr<Vertex>& vertex_ptr){
                if(auto it = vertex_id_map.find(vertex_ptr.get()); it == vertex_id_map.end()){ // 找不到？加入！
                    vertex_id_map[vertex_ptr.get()] = vertex_count++;
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
                    
                    process_vertex(i_half_edge->GetStart());
                    process_vertex(i_half_edge->GetEnd());

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);
            }


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
            deletedIdLists.clear();

            edgesMap.clear();

        }

        int GetId(const std::shared_ptr<Entity> p){
            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.second;
            }

            return -1;
        }

        TopoType GetType(const std::shared_ptr<Entity> p){

            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.first;
            }

            return TopoType::NoExist;
        }


    private:

        MarkNum(){}

        // 由指针类型更新UpdateMarkNumMap
        // TODO: 更新逻辑
        template<typename T>
        void UpdateMarkNumMap(const std::shared_ptr<T> ptr){
            TopoType topotype_name;

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
                topotype_name = TopoType::Entity;
            }

            markNumMap[ptr.get()] = {topotype_name, (capacities[topotype_name])++};
        }
    };


    struct StitchFixer2{
    public:
        std::shared_ptr<Solid> solid_ptr;
        std::vector<std::shared_ptr<HalfEdge>> poorCoedges;

        std::vector<std::vector<std::shared_ptr<HalfEdge>>> rings;

        StitchFixer2(const std::shared_ptr<Solid>& solid): solid_ptr(solid){}

        bool Start(bool call_fix){
            Clear();

            FindPoorCoedge();
            // if(call_fix) StitchPoorCoedge();
        }

        void Clear(){
            poorCoedges.clear();
            rings.clear();
        }

        void Status(){
            // TODO
        }

    private:

        /*
            调用顺序：1
            找破边，并保存到poor_coedge_vec中
        */
        void FindPoorCoedge(){
            SPDLOG_INFO("start.");

            // 先临时弄成循环遍历那样？

            for(auto f: solid_ptr->faces){
                auto lp = f->st;

                auto i_half_edge = lp->st;

                do{
                    if(i_half_edge == nullptr){
                        SPDLOG_INFO("i_half_edge is null");
                        break;
                    }

                    // 此处已经遍历到了所有halfedges
                    int partner_count = GeometryUtils::EdgePartnerCount(i_half_edge->edge);
                    if(partner_count == 1){
                        // [有效性检查]
                        if(i_half_edge->GetStart() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO START: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }
                        if(i_half_edge->GetEnd() == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO END: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }

                        if(i_half_edge->edge == nullptr){
                            SPDLOG_ERROR("poor coedge found, but NO EDGE: {}", MarkNum::GetInstance().GetId(i_half_edge));
                        }

                        // [有效性检查] END
                        
                        // 构造poor_coedge
                        poorCoedges.emplace_back((i_half_edge));

                    }

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);

            }

            SPDLOG_DEBUG("poor coedge total num: {}", static_cast<int>(poorCoedges.size()));
            SPDLOG_INFO("end.");
        }

        /*
            调用顺序：1
            找环，并保存到rings中
        */
        void FindRings(){
            SPDLOG_INFO("start.");

            std::map<std::shared_ptr<Vertex>, std::shared_ptr<HalfEdge>> st_map, ed_map;
            
            for(auto poor_coedge: poorCoedges){
                auto st = poor_coedge->GetStart();
                auto ed = poor_coedge->GetEnd();

                if(st_map.find(st) == st_map.end()){
                    st_map[st] = poor_coedge;
                }
                else{
                    SPDLOG_ERROR("st_map[st] is already exist!");
                }

                if(ed_map.find(ed) == ed_map.end()){
                    ed_map[ed] = poor_coedge;
                }
                else{
                    SPDLOG_ERROR("ed_map[st] is already exist!");
                }
            }

            std::set<std::shared_ptr<HalfEdge>> flags;

            for(int i=0;i<poorCoedges.size();i++){
                auto poor_coedge = poorCoedges[i];
                if(flags.count(poor_coedge) == 0){
                    auto i_poor_coedge = poor_coedge;

                    std::vector<std::shared_ptr<HalfEdge>> ring;
                    bool break_flag = false;

                    do{
                        if(i_poor_coedge == nullptr){
                            SPDLOG_ERROR("i_poor_coedge is nullptr");
                            break_flag = true;
                            break;

                        }
                        if(flags.count(i_poor_coedge)){
                            SPDLOG_ERROR("i_poor_coedge has visited: {}", MarkNum::GetInstance().GetId(i_poor_coedge));
                            break_flag = true;
                            break;
                        }

                        auto ed = poor_coedge->GetEnd();
                        ring.emplace_back(poor_coedge);
                        flags.insert(poor_coedge);
                    
                        i_poor_coedge = st_map[ed];

                    }while(i_poor_coedge != poor_coedge);

                    if(break_flag == false){
                        SPDLOG_INFO("ring added.");
                        rings.emplace_back(ring);
                    }else{
                        // 撤销对flags的修改
                        for(auto p: ring){
                            flags.erase(p);
                        }
                    }
                }

            }
            
            // 打印rings中已经找到的环的信息
            SPDLOG_DEBUG("rings size: {}", rings.size());

            // for(auto ring: rings){
            //     for(auto poor_coedge: ring){
            //         SPDLOG_DEBUG("");
            //     }
            // }

            SPDLOG_INFO("end.");
        }

    };

}
