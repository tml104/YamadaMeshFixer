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

    struct MarkNum{

    public:

        int entityCount;
        int solidCount;
        int faceCount;
        int loopCount;
        int halfedgeCount;
        int edgeCount;
        int vertexCount;

        std::map<Entity*, std::pair<std::string, int>> markNumMap;
        // std::map<Entity*, int> solidMap; // no used
        std::map<std::pair<int, int>, std::shared_ptr<Edge>> edgesMap;
        std::vector<std::shared_ptr<Vertex>> vertexPtrs;
        

        std::vector<std::shared_ptr<Solid>> solids;

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

            for(int i=0,j=0;i<obj_info.vertices.size();i+=3, j+=1){
                auto vertex_ptr = std::make_shared<Vertex>();
                UpdateMarkNumMap(vertex_ptr);
                vertex_ptr->pointCoord = obj_info.GetPoint(j);

                vertexPtrs.emplace_back(vertex_ptr);
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
                    edge_ptr->st = vertexPtrs[i];
                    edge_ptr->ed = vertexPtrs[j];

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
                for(int h=0;h<edge_ptr->halfEdges.size();h++){
                    edge_ptr->halfEdges[h]->partner = edge_ptr->halfEdges[(h+1) % edge_ptr->halfEdges.size()];
                }

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

            SPDLOG_INFO("Entity Count: {}", entityCount);
            SPDLOG_INFO("Solid Count: {}", solidCount);
            SPDLOG_INFO("Face Count: {}", faceCount);
            SPDLOG_INFO("Loop Count: {}", loopCount);
            SPDLOG_INFO("HalfEdge Count: {}", halfedgeCount);
            SPDLOG_INFO("Edge Count: {}", edgeCount);
            SPDLOG_INFO("Vertex Count: {}", vertexCount);

            SPDLOG_INFO("End.");
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
            for(auto vertex_ptr: vertexPtrs){
                f << "v " << vertex_ptr->pointCoord.x() << " "<< vertex_ptr->pointCoord.y() << " " << vertex_ptr->pointCoord.z() << "\n";
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

                    vertices_3_indices.emplace_back(markNumMap[i_half_edge->GetStart().get()].second);

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);

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
            markNumMap.clear();
            // solidMap.clear();
            edgesMap.clear();
            solids.clear();
            vertexPtrs.clear();

            entityCount = 0;
            solidCount = 0;
            faceCount = 0;
            loopCount = 0;
            halfedgeCount = 0;
            edgeCount = 0;
            vertexCount = 0;
        }

        int GetId(const std::shared_ptr<Entity> p){
            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.second;
            }

            return -1;
        }

        std::string GetType(const std::shared_ptr<Entity> p){

            if(auto it = markNumMap.find(p.get()); it != markNumMap.end()){
                return it->second.first;
            }

            return "NoExist";
        }


    private:

        MarkNum(){}

        // 由指针类型更新UpdateMarkNumMap
        template<typename T>
        void UpdateMarkNumMap(const std::shared_ptr<T> ptr){
            std::string typeename;
            int* counter_pointer;

            if constexpr (std::is_same_v<T, Vertex>){
                typeename = "Vertex";
                counter_pointer = &vertexCount;
            }
            else if constexpr (std::is_same_v<T, HalfEdge>){
                typeename = "HalfEdge";
                counter_pointer = &halfedgeCount;
            }
            else if constexpr (std::is_same_v<T, Edge>){
                typeename = "Edge";
                counter_pointer = &edgeCount;
            }
            else if constexpr (std::is_same_v<T, Loop>){
                typeename = "Loop";
                counter_pointer = &loopCount;
            }
            else if constexpr (std::is_same_v<T, Face>){
                typeename = "Face";
                counter_pointer = &faceCount;    
            }
            else if constexpr (std::is_same_v<T, Solid>){
                typeename = "Solid";
                counter_pointer = &solidCount;    
            }
            else{
                typeename = "Entity";
                counter_pointer = &entityCount;
            }

            markNumMap[ptr.get()] = {typeename, (*counter_pointer)++};
        }
    };


    struct PoorCoedge{
        std::shared_ptr<HalfEdge> halfEdge;
        Coordinate midPoint;

        PoorCoedge() {}

        PoorCoedge(const std::shared_ptr<HalfEdge>& halfEdge): halfEdge(halfEdge){
            midPoint = (halfEdge->GetStart()->pointCoord + halfEdge->GetEnd()->pointCoord) /2;
        }

        // 计算与给定PoorCoedge的匹配分数
        // 这里和brep中的计算可能不太一样?
        double CalculateScore(const PoorCoedge& other_poor_coedge) const {
            double score = 0.0;
           	bool valid_flag = true;
            
            T_NUM distance = 0.0;

            // 中点
            distance = midPoint.Distance(other_poor_coedge.midPoint);
            if(distance > EPSLION1) {valid_flag = false;}
            score += distance;

            // 起始点 与 终点 距离
            distance = halfEdge->GetStart()->pointCoord.Distance(other_poor_coedge.halfEdge->GetEnd()->pointCoord);
            if(distance > EPSLION1) {valid_flag = false;}
            score += distance;

            // 终点 与 起始点 距离
            distance = halfEdge->GetEnd()->pointCoord.Distance(other_poor_coedge.halfEdge->GetStart()->pointCoord);
            if(distance > EPSLION1) {valid_flag = false;}
            score += distance;

            // 对距离：归一化后计算平均分数（此时应该是一个小于1的数值），然后加权
            score /= EPSLION1;
            score /= 3;
            
            // 计算方向向量所构成之夹角

            auto vec1 = (halfEdge->GetStart()->pointCoord - halfEdge->GetEnd()->pointCoord);
            auto vec2 = (other_poor_coedge.halfEdge->GetStart()->pointCoord - other_poor_coedge.halfEdge->GetEnd()->pointCoord);
            vec1.Normalize();
            vec2.Normalize();

            double dot_result = vec1.Dot(vec2);

            if(dot_result > 0){
                valid_flag = false;
            }
            else{
            	dot_result = -dot_result;
                if(dot_result < EPSLION2){
                    valid_flag = false;
                }
            }

            if(valid_flag){
                return score;
            }

            return -1.0;
        }
    };

    struct MatchTree{

        struct MatchTreeNode{
            Coordinate minRangePoint;
            Coordinate maxRangePoint;

            MatchTreeNode* leftNode;
            MatchTreeNode* rightNode;

            int splitDim;
            bool isLeaf;
            PoorCoedge leafPoorCoedge;

            MatchTreeNode(): leftNode(nullptr), rightNode(nullptr), splitDim(-1), isLeaf(false), leafPoorCoedge(){
                for(int i=0;i<3;i++){
                    minRangePoint[i] = MAXVAL;
                    maxRangePoint[i] = MINVAL;
                }
            }

            ~MatchTreeNode(){
                if(leftNode){
                    delete leftNode;
                    leftNode = nullptr;
                }

                if(rightNode){
                    delete rightNode;
                    rightNode = nullptr;
                }
            }
        };

        MatchTreeNode* root;

        MatchTree(): root(nullptr){}

        ~MatchTree(){
            if(this->root){
                delete root;
            }
            root = nullptr;
        }

		void ConstructTree(std::vector<PoorCoedge>& poor_coedge_vec){
            
            // 叶子节点
            auto update_leaf_node = [&](PoorCoedge leaf_coedge, int now_dim)->MatchTree::MatchTreeNode * {
                MatchTree::MatchTreeNode *node = new MatchTree::MatchTreeNode();

                node->isLeaf = true;
                node->leafPoorCoedge = leaf_coedge; // 复制poorCoedge
                node->splitDim = now_dim;
    
                // boundary is set to midpoint
                // for (int i = 0; i < 3; i++)
                // {
                //     node->minRangePoint[i] = leaf_coedge.midPoint[i];
                //     node->maxRangePoint[i] = leaf_coedge.midPoint[i];
                // }
                node->minRangePoint = leaf_coedge.midPoint;
                node->maxRangePoint = leaf_coedge.midPoint;

                return node;
            };

            // 非叶子节点
            auto update_now_node = [&] (MatchTree::MatchTreeNode *left, MatchTree::MatchTreeNode *right, int now_dim) -> MatchTree::MatchTreeNode* {
                MatchTree::MatchTreeNode* node = new MatchTree::MatchTreeNode();
                node->splitDim = now_dim;

                if(left){
                    node->minRangePoint = node->minRangePoint.Min(left->minRangePoint);
                    node->maxRangePoint = node->maxRangePoint.Max(left->maxRangePoint);
                    node->leftNode = left;
                }

                if(right){
                    node->minRangePoint = node->minRangePoint.Min(right->minRangePoint);
                    node->maxRangePoint = node->maxRangePoint.Max(right->maxRangePoint);
                    node->rightNode = right;
                }

                return node;
            };

            // 递归调用
            std::function<MatchTree::MatchTreeNode*(std::vector<PoorCoedge>&, int, int, int)> recursive_construct = [&] (std::vector<PoorCoedge>& poor_coedge_vec, int now_dim, int l, int r) -> MatchTree::MatchTreeNode* {
                if(l == r){
                    return update_leaf_node(poor_coedge_vec[l], now_dim);
                }

                if(l>r){
                    return nullptr;
                }

                int mid = (l+r)>>1;

                // 利用nth_element划分poor_coedge_vec
                std::nth_element(poor_coedge_vec.begin()+l, poor_coedge_vec.begin()+mid, poor_coedge_vec.begin()+r+1, [&](const PoorCoedge& a, const PoorCoedge& b){
                    return a.midPoint[now_dim] < b.midPoint[now_dim];
                });

                MatchTree::MatchTreeNode* left_node = recursive_construct(poor_coedge_vec, (now_dim+1)%3, l, mid);
                MatchTree::MatchTreeNode* right_node = recursive_construct(poor_coedge_vec, (now_dim+1)%3, mid+1, r);
                

                return update_now_node(left_node, right_node, now_dim);
            };

            this->root = recursive_construct(poor_coedge_vec, 0, 0, static_cast<int>(poor_coedge_vec.size())-1);
        }

		void DeleteTree(){
            if(this->root){
                delete this->root;
            }
            root = nullptr;

            SPDLOG_INFO("DeleteTree Done.");
        }

        std::vector<std::pair<PoorCoedge, double>> Match(
            PoorCoedge poor_coedge, 
            const std::set<std::shared_ptr<HalfEdge>>& found_coedge_set, 
            bool dont_stitch_coincident
        ){
            std::vector<std::pair<PoorCoedge, double>> match_res_vec;

            // 检查子树是否符合剪枝条件。若应该剪枝则返回false
            // 		注意这里的子树剪枝条件会根据EPSLION1放松一些
            auto check_subtree_valid = [&] (MatchTree::MatchTreeNode *now_root) -> bool {

                for(int i=0;i<3;i++){
                    // 取反（下面是符合条件之内容）
                    if(!(
                        (now_root->minRangePoint[i] - EPSLION1 < poor_coedge.midPoint[i])
                        &&
                        ( poor_coedge.midPoint[i] < now_root->maxRangePoint[i] + EPSLION1 )
                    )){
                        return false;
                    }
                }

                return true;
            };

            // 取得匹配分数
            auto get_match_score = [&](MatchTree::MatchTreeNode* now_root) -> double {
                PoorCoedge &poor_coedge2 = now_root->leafPoorCoedge;
                return poor_coedge.CalculateScore(poor_coedge2);
            };

            // 相同环检查
            auto check_same_loop_from_coedges = [&] (const std::shared_ptr<HalfEdge>& he1, const std::shared_ptr<HalfEdge>& he2) -> bool {
                if(he1->loop && he2->loop && (he1->loop == he2->loop)){
                    return true;
                }
                return false;
            };

            // 匹配取更新match_res_vec
            std::function<void(MatchTree::MatchTreeNode* )> recursive_match = [&](MatchTree::MatchTreeNode *now_root){

                // 递归的边界条件
                if(now_root->isLeaf){
                    if(now_root->leafPoorCoedge.halfEdge == poor_coedge.halfEdge){
                        SPDLOG_DEBUG("Skip self match.");
                        return ;
                    }

                    if(found_coedge_set.count(poor_coedge.halfEdge)){
                        SPDLOG_DEBUG("Skip matched halfedge.");
                        return ;
                    }

                    if(check_same_loop_from_coedges(now_root->leafPoorCoedge.halfEdge, poor_coedge.halfEdge)){
                        SPDLOG_DEBUG("Skip same face coedges.");
                        return ;
                    }

                    if(dont_stitch_coincident && GeometryUtils::GeometryCoincidentEdge(poor_coedge.halfEdge->edge, now_root->leafPoorCoedge.halfEdge->edge)){
                        SPDLOG_DEBUG("Skip coincident coedge.");
                        return ;
                    }

                	double score = get_match_score(now_root);

                    if(score > 0){
                        match_res_vec.emplace_back(now_root->leafPoorCoedge, score);
                    }

                    return;
                }

                if(!check_subtree_valid(now_root)){
                    return ;
                }

                if(now_root->leftNode){
                    recursive_match(now_root->leftNode);
                }

                if(now_root->rightNode){
                    recursive_match(now_root->rightNode);
                }
            
            };

            recursive_match(this->root);

            std::sort(match_res_vec.begin(), match_res_vec.end(), [&](const std::pair<PoorCoedge, double>& a, const std::pair<PoorCoedge, double>& b){
                return a.second < b.second;
            });

            return match_res_vec;
        }

    };

    struct StitchFixer{
    public:
        std::shared_ptr<Solid> solid_ptr;
        bool dontStitchCoincident;

        std::vector<PoorCoedge> poorCoedges;
        std::set<std::shared_ptr<HalfEdge>> foundCoedges;
        std::vector<std::pair<PoorCoedge, PoorCoedge>> poorCoedgePairs;

        MatchTree matchTree;

        StitchFixer(const std::shared_ptr<Solid>& solid): solid_ptr(solid){}

        bool Start(bool call_fix, bool dont_stitch_coincident){
            Clear();
            this->dontStitchCoincident = dont_stitch_coincident;

            FindPoorCoedge();
            MatchPoorCoedge();
            RearrangePoorCoedge();
            if(call_fix) StitchPoorCoedge();
        }

        void Clear(){
            dontStitchCoincident = true;
            poorCoedges.clear();
            foundCoedges.clear();
            poorCoedgePairs.clear();

            matchTree.DeleteTree();
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
                        poorCoedges.emplace_back(PoorCoedge(i_half_edge));

                    }

                    i_half_edge = i_half_edge->next;
                }while(i_half_edge && i_half_edge != lp->st);

            }

            SPDLOG_DEBUG("poor coedge total num: {}", static_cast<int>(poorCoedges.size()));
        }

        /*
            调用顺序：2
            匹配每条边，并且构建KDTree进行加速
            这个匹配是贪心地匹配的（也就是说一旦匹配成功就把它放进对应的found_coedge_set，后续不再参与匹配），可能之后要改一下
        */
        void MatchPoorCoedge(){
            matchTree.ConstructTree(poorCoedges);
            for(auto poor_coedge: poorCoedges){
                // 已经找过
                if(foundCoedges.count(poor_coedge.halfEdge)){
                    continue;
                }

                auto match_poor_coedge_vec = matchTree.Match(poor_coedge, foundCoedges, dontStitchCoincident);
                if(match_poor_coedge_vec.size()){
                    auto first_match = match_poor_coedge_vec.front();

                    poorCoedgePairs.emplace_back(poor_coedge, first_match.first);

                    foundCoedges.insert(poor_coedge.halfEdge);
                    foundCoedges.insert(first_match.first.halfEdge);

                }
            }

            // [Debug] 打印配对边信息
            for(auto poorCoedgePair: poorCoedgePairs){
                SPDLOG_DEBUG("Match pair: (coedge id pair) (edge id pair): ({}, {}) ({}, {})",
                    MarkNum::GetInstance().GetId(poorCoedgePair.first.halfEdge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.second.halfEdge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.first.halfEdge->edge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.second.halfEdge->edge)
                );
            }
        }

        /*
            调用顺序：3
            重排序
        */
        void RearrangePoorCoedge(){
            // 对 poorCoedgePairs 中的匹配边对进行重排序，使得其符合修复顺序
            /*
                按照如下顺序排序，然后放入poor_coedge_pair_vec2中
                (1) 先缝合不会删边的
                (2) 再缝合会删非已经匹配边（正常边，非匹配的独立缺边）的
                (3) 最后全部缝合
	        */

            decltype(poorCoedgePairs) poor_coedge_pair_vec2;

            // 对poor_coedge_pair_vec2部分重排序：poor_coedge_pair_vec2中保存的是（第一阶段重新排序后的）若干对(pair)已经配对的破边，现在根据破边的长度和对第一阶段重新排序中各个部分的配对破边做第二阶段排序
            // （注意lr满足左闭右开）
            auto partial_sort_for_poor_coedge_pair_vec2 = [&](decltype(poor_coedge_pair_vec2)& poor_coedge_pair_vec2, int l, int r) {
                std::sort(
                    poor_coedge_pair_vec2.begin() + l,
                    poor_coedge_pair_vec2.begin() + r,
                    [&](const std::pair<PoorCoedge, PoorCoedge>& pair_a, const std::pair<PoorCoedge, PoorCoedge>& pair_b) -> bool {
                        double pair_a_score = pair_a.first.CalculateScore(pair_a.second);
                        double pair_b_score = pair_b.first.CalculateScore(pair_b.second);

                        return pair_a_score < pair_b_score; // 目前这个分数只和距离有关（距离越小越优先），因此这样就是让距离小的优先处理
                    }
                );
            };

            auto rearrange_poor_coedge_pairs = [&](decltype(poor_coedge_pair_vec2)& poor_coedge_pair_vec2){
                // 标记向量：如果对应配对边组合被标记为true，则跳过它们，并且将它们从found_coedge_set集合中移除
                std::vector<bool> poor_coedge_pair_vec_flag(poorCoedgePairs.size()); 

                // (2.0) 警告交叉连接的情况
                auto f2_0 = [&] (const std::pair<PoorCoedge, PoorCoedge>& poor_coedge_pair, const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2){
                    // std::vector<EDGE*> edges = edges_data.FindEdgesBetweenVertices(v1, v2);
                    auto edge = MarkNum::GetInstance().FindEdgeBetweenVertices(v1, v2);

                    // 此判断排除这个交叉边和已配对边相同的情况（这个有啥必要吗？）
                    if(edge && edge != poor_coedge_pair.first.halfEdge->edge && edge != poor_coedge_pair.second.halfEdge->edge){
                        SPDLOG_WARN("(2.0) Warn: Connect between v0 v01 or v1 v11: (poor_coedge_pair.first.coedge: {}, edge: {}), (poor_coedge_pair.second.coedge: {}, edge: {}) (v1: {}, v2: {}, e: {})",
                            MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge),
                            MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge->edge),
                            MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge),
                            MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge->edge),
                            MarkNum::GetInstance().GetId(v1),
                            MarkNum::GetInstance().GetId(v2),
                            MarkNum::GetInstance().GetId(edge)
                        );
                    }
                };

                for(int i=0;i<poorCoedgePairs.size();i++){
                    if(poor_coedge_pair_vec_flag[i]) {continue;}

                    auto &poor_coedge_pair = poorCoedgePairs[i]; // 取得已经配对的一对

                    auto v0 = poor_coedge_pair.first.halfEdge->GetStart();
                    auto v1 = poor_coedge_pair.first.halfEdge->GetEnd();
                    auto v01 = poor_coedge_pair.second.halfEdge->GetStart();
                    auto v11 = poor_coedge_pair.second.halfEdge->GetEnd();

                    f2_0(poor_coedge_pair, v0, v01);
                    f2_0(poor_coedge_pair, v1, v11);
                }

                // (2.1) 顺序1：无连接
                for(int i=0;i<poorCoedgePairs.size();i++){
                    if(poor_coedge_pair_vec_flag[i]) {continue;}

                    auto &poor_coedge_pair = poorCoedgePairs[i]; // 取得已经配对的一对

                    auto v0 = poor_coedge_pair.first.halfEdge->GetStart();
                    auto v1 = poor_coedge_pair.first.halfEdge->GetEnd();
                    auto v01 = poor_coedge_pair.second.halfEdge->GetStart();
                    auto v11 = poor_coedge_pair.second.halfEdge->GetEnd();

                    if(MarkNum::GetInstance().FindEdgeBetweenVertices(v0, v11) == nullptr 
                        && MarkNum::GetInstance().FindEdgeBetweenVertices(v1, v01) == nullptr 
                        && MarkNum::GetInstance().FindEdgeBetweenVertices(v1, v11) == nullptr 
                        && MarkNum::GetInstance().FindEdgeBetweenVertices(v0, v01) == nullptr 
                    ){
                        poor_coedge_pair_vec2.emplace_back(poor_coedge_pair);
                        poor_coedge_pair_vec_flag[i] = true;
                    }
                }
                // -> 局部重排序
                int poor_coedge_pair_vec2_size_2_1 = poor_coedge_pair_vec2.size();
                partial_sort_for_poor_coedge_pair_vec2(poor_coedge_pair_vec2, 0, poor_coedge_pair_vec2_size_2_1);

                // (2.2) 顺序2：连接非已经匹配边
                auto f2_2 = [&](const std::pair<PoorCoedge, PoorCoedge>& poor_coedge_pair, const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2, bool& flag) {
                    auto edge = MarkNum::GetInstance().FindEdgeBetweenVertices(v1, v2);

                    //遍历这个边的所有coedge，如果存在coedge在Stitch::Singleton::found_coedge_set中则说明这不是一个非已匹边
                    if(edge){
                        for(auto i_half_edge: edge->halfEdges){
                            if(foundCoedges.count(i_half_edge)){
                                flag = false;
                                break;
                            }
                        }
                    }
                };

                for(int i=0;i<poorCoedgePairs.size();i++){
                    if(poor_coedge_pair_vec_flag[i]) {continue;}
                    auto &poor_coedge_pair = poorCoedgePairs[i]; // 取得已经配对的一对

                    auto v0 = poor_coedge_pair.first.halfEdge->GetStart();
                    auto v1 = poor_coedge_pair.first.halfEdge->GetEnd();
                    auto v01 = poor_coedge_pair.second.halfEdge->GetStart();
                    auto v11 = poor_coedge_pair.second.halfEdge->GetEnd();

                    bool flag = true;

                    f2_2(poor_coedge_pair, v0, v11, flag);
                    f2_2(poor_coedge_pair, v1, v01, flag);

                    if(flag){
                        poor_coedge_pair_vec2.emplace_back(poor_coedge_pair);
                        poor_coedge_pair_vec_flag[i] = true;
                    }
                }
                // -> 局部重排序
                int poor_coedge_pair_vec2_size_2_2 = poor_coedge_pair_vec2.size();
                partial_sort_for_poor_coedge_pair_vec2(poor_coedge_pair_vec2, poor_coedge_pair_vec2_size_2_1, poor_coedge_pair_vec2_size_2_2);

                // (2.3) 顺序3：剩下的情况
                for(int i=0;i<poorCoedgePairs.size();i++){
                    if(poor_coedge_pair_vec_flag[i]) {continue;}
                    auto &poor_coedge_pair = poorCoedgePairs[i]; // 取得已经配对的一对

                    poor_coedge_pair_vec2.emplace_back(poor_coedge_pair);
                }
                // -> 局部重排序
                int poor_coedge_pair_vec2_size_2_3 = poor_coedge_pair_vec2.size();
		        partial_sort_for_poor_coedge_pair_vec2(poor_coedge_pair_vec2, poor_coedge_pair_vec2_size_2_2, poor_coedge_pair_vec2_size_2_3);

            };

            rearrange_poor_coedge_pairs(poor_coedge_pair_vec2);

            poorCoedgePairs = std::move(poor_coedge_pair_vec2);

            // [Debug] 重排序后的配对边信息
            for(auto poorCoedgePair: poorCoedgePairs){
                SPDLOG_DEBUG("Match pair AFTER REARRANGE: (coedge id pair) (edge id pair): ({}, {}) ({}, {})",
                    MarkNum::GetInstance().GetId(poorCoedgePair.first.halfEdge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.second.halfEdge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.first.halfEdge->edge),
                    MarkNum::GetInstance().GetId(poorCoedgePair.second.halfEdge->edge)
                );
            }
        }

        /*
            调用顺序：4
            对匹配的poor coedge对修改拓扑
            输入：目前poor_coedge_pair_vec已经维护完成
        */
        void StitchPoorCoedge(){

            // 3. 开始修复：遍历poor_coedge_pair_vec2（重排序后的候选破边对集合），然后依次尝试修改对应破边的各种指针以及顶点指针
            // 规定v0 v1所在边是被与另一个合并的

            for(int i=0;i<poorCoedgePairs.size();i++){
                auto &poor_coedge_pair = poorCoedgePairs[i];
                
                // [debug] 打印当前修复的poorcoedge pair的信息
                SPDLOG_DEBUG("Stitch Fix for poor_coedge_pair[{}]: (poor_coedge_pair.first.coedge: {}, edge: {}), (poor_coedge_pair.second.coedge: {}, edge: {})",
                    i,
                    MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge),
                    MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge->edge),
                    MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge),
                    MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge->edge)
                );

                // 注意这里必须判断要修复的边是不是还在found_coedge_set里面。如果因为先前的步骤被删掉对应边了就不能再做这个修复了
                if(foundCoedges.count(poor_coedge_pair.first.halfEdge) == 0 || foundCoedges.count(poor_coedge_pair.second.halfEdge) == 0){

                    SPDLOG_DEBUG("poor_coedge_pair SKIP fixing, as they not longer in found_coedge_set: (poor_coedge_pair.first.coedge: {}, edge: {}), (poor_coedge_pair.second.coedge: {}, edge: {})",
                        MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge),
                        MarkNum::GetInstance().GetId(poor_coedge_pair.first.halfEdge->edge),
                        MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge),
                        MarkNum::GetInstance().GetId(poor_coedge_pair.second.halfEdge->edge)
                    );
                    continue;
                }

                auto v0 = poor_coedge_pair.first.halfEdge->GetStart();
                auto v1 = poor_coedge_pair.first.halfEdge->GetEnd();
                auto v01 = poor_coedge_pair.second.halfEdge->GetStart();
                auto v11 = poor_coedge_pair.second.halfEdge->GetEnd();

                // 3.1 先改指针
                poor_coedge_pair.first.halfEdge->partner = poor_coedge_pair.second.halfEdge;
                poor_coedge_pair.second.halfEdge->partner = poor_coedge_pair.first.halfEdge;
                poor_coedge_pair.second.halfEdge->edge = poor_coedge_pair.first.halfEdge->edge;
                poor_coedge_pair.first.halfEdge->edge->AddHalfEdge(poor_coedge_pair.second.halfEdge);
                poor_coedge_pair.second.halfEdge->sense = !(poor_coedge_pair.first.halfEdge->sense);

                // 3.2 修改所有边的v11, v01为v0, v1（如果原本v0==v11等那就跳过）
                for(auto it: MarkNum::GetInstance().edgesMap){
                    auto e = it.second;
                    if(v0 != v11){
                        if(e->st == v11){
                            SPDLOG_DEBUG("(3.2) Set e->st from v11 to v0: {} {} {}",
                                MarkNum::GetInstance().GetId(e),
                                MarkNum::GetInstance().GetId(v11),
                                MarkNum::GetInstance().GetId(v0)
                            );
                            e->st = v0;
                        }

                        if(e->ed == v11){
                            SPDLOG_DEBUG("(3.2) Set e->ed from v11 to v0: {} {} {}",
                                MarkNum::GetInstance().GetId(e),
                                MarkNum::GetInstance().GetId(v11),
                                MarkNum::GetInstance().GetId(v0)
                            );
                            e->ed = v0;
                        }
                    }

                    if(v1 != v01){
                        if(e->st == v01){
                            SPDLOG_DEBUG("(3.2) Set e->st from v01 to v1: {} {} {}",
                                MarkNum::GetInstance().GetId(e),
                                MarkNum::GetInstance().GetId(v01),
                                MarkNum::GetInstance().GetId(v1)
                            );
                            e->st = v1;
                        }

                        if(e->ed == v01){
                            SPDLOG_DEBUG("(3.2) Set e->ed from v01 to v1: {} {} {}",
                                MarkNum::GetInstance().GetId(e),
                                MarkNum::GetInstance().GetId(v01),
                                MarkNum::GetInstance().GetId(v1)
                            );
                            e->ed = v1;
                        }
                    }
                }

                // TODO: 这个地方到底要不要做原先3.3的删边操作实际上有待考察

                // 3.3 检查v0 v11（v1 v01）的相连情况，如果有相连就删除对应边（修改链表，相当于从loop中删除对应边）
                // auto f3_3 = [&](const std::shared_ptr<Vertex>& v1, const std::shared_ptr<Vertex>& v2){
                //     auto e = MarkNum::GetInstance().FindEdgeBetweenVertices(v1, v2);

                //     // 删边
                //     if(e){

                //     }
                // };

                // 3.3 执行

                // 改几何部分跳过
            }
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
