#include <map>
#include <iostream>
#include <cassert>
#include <cmath>
#include <random>

#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>
#include "eigenheaders.h"
#include "Scenes.h"
#include "Options.h"
#include "MeshIO.h"
#include "Sim.h"
#include "SZHSolver.h"

class OrderComp
{
public:
    bool operator () (const std::pair<int, double> & o1, const std::pair<int, double> & o2) const { return o1.second < o2.second; }
};

// subdivide every triangle in the mesh into four triangles, by introducing three new vertices at edge midpoints
// after subdivision, vs will be expanded (retaining all original vertices), while fs and ls will be replaced (no original faces remain)
// if r is positive, new vertices will be projected onto the sphere centered at center, with a radius of r (for ICO sphere generation)
void subdivide(const Vec3d & center, double r, std::vector<Vec3d> & vs, std::vector<Vec3i> & fs, std::vector<Vec2i> & ls)
{
    std::vector<Vec3i> new_fs;
    std::vector<Vec2i> new_ls;
    std::map<std::pair<int, int>, int> new_vs_map;
    
    // map edge-midpoint to new vertices
    for (size_t j = 0; j < fs.size(); j++)
    {
        Vec3i & f = fs[j];
        int v0 = f[0];
        int v1 = f[1];
        int v2 = f[2];
        
        std::pair<int, int> p01 = (v0 < v1 ? std::pair<int, int>(v0, v1) : std::pair<int, int>(v1, v0));
        std::pair<int, int> p12 = (v1 < v2 ? std::pair<int, int>(v1, v2) : std::pair<int, int>(v2, v1));
        std::pair<int, int> p20 = (v2 < v0 ? std::pair<int, int>(v2, v0) : std::pair<int, int>(v0, v2));
        
        new_vs_map[p01] = 0;
        new_vs_map[p12] = 0;
        new_vs_map[p20] = 0;
    }
    
    // create the new vertices
    for (std::map<std::pair<int, int>, int>::iterator j = new_vs_map.begin(); j != new_vs_map.end(); j++)
    {
        j->second = vs.size();
        if (r > 0)
            vs.push_back(((vs[j->first.first] + vs[j->first.second]) / 2 - center).normalized() * r + center);
        else
            vs.push_back((vs[j->first.first] + vs[j->first.second]) / 2);
    }
    
    // triangulate
    for (size_t j = 0; j < fs.size(); j++)
    {
        Vec3i & f = fs[j];
        int v0 = f[0];
        int v1 = f[1];
        int v2 = f[2];
        
        std::pair<int, int> p01 = (v0 < v1 ? std::pair<int, int>(v0, v1) : std::pair<int, int>(v1, v0));
        std::pair<int, int> p12 = (v1 < v2 ? std::pair<int, int>(v1, v2) : std::pair<int, int>(v2, v1));
        std::pair<int, int> p20 = (v2 < v0 ? std::pair<int, int>(v2, v0) : std::pair<int, int>(v0, v2));
        int nv01 = new_vs_map[p01];
        int nv12 = new_vs_map[p12];
        int nv20 = new_vs_map[p20];
        
        Vec2i & l = ls[j];
        new_fs.push_back(Vec3i(v0, nv01, nv20));   new_ls.push_back(l);
        new_fs.push_back(Vec3i(nv01, v1, nv12));   new_ls.push_back(l);
        new_fs.push_back(Vec3i(nv20, nv12, v2));   new_ls.push_back(l);
        new_fs.push_back(Vec3i(nv12, nv20, nv01)); new_ls.push_back(l);
    }
    
    fs = new_fs;
    ls = new_ls;
}
    

SZHSolver * SZHScenes::sceneInputData(Sim * sim, std::vector<LosTopos::Vec3d> & vs, std::vector<LosTopos::Vec3st> & fs, std::vector<LosTopos::Vec2i> & ls, std::vector<size_t> & cv, std::vector<Vec3d> & cx,const std::string inputdata_dir)
{
    int num_bubbles=-1;

    int N= Options::intValue("num_subdivision");

    using namespace Eigen;
    MatrixXd V;
    MatrixXi F;
    
    std::string meshfile;
    std::string labelfile;
    std::string constrained_file;
    
    std::string sub_scene=Options::strValue("sub_scene");
    
    meshfile=sub_scene+".obj";
    labelfile=sub_scene+"_flabel.txt";
    constrained_file=sub_scene+"_constvertex.txt";
    std::ifstream mesh_ifs(inputdata_dir+meshfile);
    igl::read_triangle_mesh(inputdata_dir+meshfile, V, F);
    std::vector<Vec3d> v;
    std::vector<Vec3i> f;
    std::vector<Vec2i> l;
    
    int nv=V.rows();
    for(int vi=0;vi<nv;++vi){
        v.push_back(V.row(vi));
        
    }
    int nt=F.rows();

    for(int fi=0;fi<nt;++fi){
        f.push_back(F.row(fi));
    }
    
    std::ifstream label_ifs(inputdata_dir+labelfile);
    if(label_ifs){

        std::string str;
        getline(label_ifs,str);
        
        getline(label_ifs,str);
        int ntl=std::stoi(str);
        assert(nt=ntl);

        Vec2i flabel;
        int fcount=0;
        while(getline(label_ifs,str)){
            std::string region_str;
            std::istringstream stream(str);
            
            //seperate line by ' '.
            int region_num=0;
            while(getline(stream,region_str,' ')){
                
                int region=std::stoi(region_str);
                //cout<<region<<",";
                // FLabel(fcount,region_num)=region-1;
                flabel[region_num]=region;
                ++region_num;
            }
            
            l.push_back(flabel);
            ++fcount;
        }
    }
    for (int i = 0; i < N; i++)
        subdivide(Vec3d(0, 0, 0), 0, v, f, l);
    vs.resize(v.size());
    fs.resize(f.size());
    ls.resize(l.size());
    for (size_t i = 0; i < v.size(); i++)
        vs[i] = LosTopos::Vec3d (v[i][0], v[i][1], v[i][2]);
    for (size_t i = 0; i < f.size(); i++)
        fs[i] = LosTopos::Vec3st(f[i][0], f[i][1], f[i][2]);
    for (size_t i = 0; i < l.size(); i++)
        ls[i] = LosTopos::Vec2i (l[i][0], l[i][1]);
    return new SZHSolver(vs,fs,ls,cv,cx,num_bubbles);
}



void SZHScenes::burstBubbles(double dt, Sim * sim, SZHSolver * hgf)
{
    
    auto burst_a_bubble=[hgf](){
        std::cout << "Bursting a random bubble now." << std::endl;
        
        std::set<int> burstable_regions_set;
        for (size_t i = 0; i < hgf->mesh().nt(); i++)
        {
            LosTopos::Vec2i l = hgf->mesh().get_triangle_label(i);
            if (l[0] == 0 || l[1] == 0)
                burstable_regions_set.insert(l[0] == 0 ? l[1] : l[0]);
        }
        
        std::vector<int> burstable_regions;
        burstable_regions.assign(burstable_regions_set.begin(), burstable_regions_set.end());
        std::cout << "Eligible regions: ";
        for (size_t i = 0; i < burstable_regions.size(); i++)
            std::cout << burstable_regions[i] << " ";
        std::cout << std::endl;

        int region_to_burst = burstable_regions[rand() % burstable_regions.size()];
        std::cout << "Region to be bursted: " << region_to_burst << std::endl;

        int A = region_to_burst;
        std::set<int> Bs;
        for (size_t i = 0; i < hgf->mesh().nt(); i++)
        {
            LosTopos::Vec2i l = hgf->mesh().get_triangle_label(i);
            if (l[0] == region_to_burst || l[1] == region_to_burst)
                Bs.insert(l[0] == region_to_burst ? l[1] : l[0]);
        }

        for (size_t i = 0; i < hgf->mesh().nt(); i++)
        {
            LosTopos::Vec2i l = hgf->mesh().get_triangle_label(i);
            if (l[0] == region_to_burst || l[1] == region_to_burst)
            {
                int lother = (l[0] == region_to_burst ? l[1] : l[0]);
                if (lother == 0)
                    hgf->surfTrack()->remove_triangle(i);
                else
                    hgf->mesh().set_triangle_label(i, (l[0] == region_to_burst ? LosTopos::Vec2i(0, l[1]) : LosTopos::Vec2i(l[0], 0)));
            }
        }
        
        hgf->surfTrack()->defrag_mesh_from_scratch(hgf->m_constrained_vertices);
        
        std::cout << "Bubble bursted." << std::endl;
        
    };
    burst_a_bubble();
}
void SZHScenes::pullBubbles(double dt, Sim * sim, SZHSolver * hgf)
{
    Vec3d center0(0, 0, 0);
    int counter0 = 0;
    Vec3d center1(0, 0, 0);
    int counter1 = 0;
    
    for (size_t i = 0; i < hgf->m_constrained_positions.size(); i++)
    {
        if (hgf->m_constrained_positions[i].x() > 0)
        {
            center0 += hgf->m_constrained_positions[i];
            counter0++;
        } else
        {
            center1 += hgf->m_constrained_positions[i];
            counter1++;
        }
    }
    
    center0 /= counter0;
    center1 /= counter1;
    
    Vec3d dir = (center0 - center1).normalized();
    
    double pulling_velocity=0.1;//0.02;
    
    for (size_t i = 0; i < hgf->m_constrained_vertices.size(); i++)
        hgf->m_constrained_positions[i] += (hgf->m_constrained_positions[i].x() > 0 ? 1 : -1) * dir * pulling_velocity * dt;
}

void SZHScenes::volume_change(double dt, Sim * sim, SZHSolver * hgf){
    
    if(hgf->blowing_bubble0){
        
        hgf->initialVolumes[0]+=0.01;
        
    }
}

