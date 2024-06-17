//
//  Sim.cpp
//
//  Fang Da 2014

#include <sstream>
#include <iomanip>
#include <chrono>
#include "Sim.h"
#include "Options.h"
#include "MeshIO.h"
#include <cmath>
#include "eigenheaders.h"
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
Sim::Sim(bool verbose) :
m_verbose(verbose),
m_scene("unspecified"),
m_output_directory(""),
hgf(NULL),
m_dt(0),
m_time(0),
m_frameid(0),
m_finished(false),
m_nearest_vertex(-1),
m_nearest_edge(-1),
m_nearest_face(-1)
{
    
}

Sim::~Sim()
{
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  General initialization of a simulation
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Sim::init(const std::string & option_file,const std::string inputdata_dir)
{
    
    // declare and load the options
    Options::addStringOption ("scene", "T1");
    Options::addStringOption ("load-dir", "");
    Options::addDoubleOption ("time-step", 0.01);
    Options::addDoubleOption ("simulation-time", 1.0);
    Options::addBooleanOption("implicit_scheme", false);
    Options::addBooleanOption("RK4-velocity-integration", false);
    Options::addDoubleOption ("smoothing-coef", 0.0);
    Options::addDoubleOption ("damping-coef", 1.0);
    Options::addBooleanOption ("sparse", false);

    Options::addBooleanOption ("logging_time", true);
    Options::addBooleanOption ("logging_detailed_time", false);

    Options::addBooleanOption("output-png", true);
    Options::addIntegerOption("output-png-every-n-frames", 0);     // 0 means synching with simulation frame rate (equivalent to 1).
    Options::addBooleanOption("output-mesh", true);
    Options::addIntegerOption("output-mesh-every-n-frames", 0);    // 0 means synching with simulation frame rate (equivalent to 1).
    Options::addBooleanOption("output-obj", false);
    Options::addIntegerOption("output-obj-every-n-frames", 0);     // 0 means synching with simulation frame rate (equivalent to 1).
    Options::addDoubleOption ("remeshing-resolution", 0.1);
    Options::addIntegerOption("remeshing-iterations", 1);
    
    Options::addDoubleOption ("lostopos-collision-epsilon-fraction", 1e-4);       // lostopos collision epsilon (fraction of mean edge length)
    Options::addDoubleOption ("lostopos-merge-proximity-epsilon-fraction", 0.02); // lostopos merge proximity epsilon (fraction of mean edge length)
    Options::addBooleanOption("lostopos-perform-smoothing", false);               // whether or not to perform smoothing
    Options::addDoubleOption ("lostopos-max-volume-change-fraction", 1e-4);       // maximum allowed volume change during a remeshing operation (fraction of mean edge length cubed)
    Options::addDoubleOption ("lostopos-min-triangle-angle", 3.0);                // min triangle angle (in degrees)
    Options::addDoubleOption ("lostopos-max-triangle-angle", 177.0);              // max triangle angle (in degrees)
    Options::addDoubleOption ("lostopos-large-triangle-angle-to-split", 160.0);   // threshold for large angles to be split
    Options::addDoubleOption ("lostopos-min-triangle-area-fraction", 0.02);       // minimum allowed triangle area (fraction of mean edge length squared)
    Options::addBooleanOption("lostopos-t1-transition-enabled", true);            // whether t1 is enabled
    Options::addDoubleOption ("lostopos-t1-pull-apart-distance-fraction", 0.1);   // t1 pull apart distance (fraction of mean edge legnth)
    Options::addBooleanOption("lostopos-smooth-subdivision", false);              // whether to use smooth subdivision during remeshing
    Options::addBooleanOption("lostopos-allow-non-manifold", true);               // whether to allow non-manifold geometry in the mesh
    Options::addBooleanOption("lostopos-allow-topology-changes", true);           // whether to allow topology changes
    
    Options::addIntegerOption("num_subdivision", 0);
    Options::addIntegerOption("mesh-size-n", 2);
    Options::addIntegerOption("mesh-size-m", 2);
    
    Options::addBooleanOption("auto-burst", true);
    Options::addDoubleOption("auto-burst-interval", 20.0);
    Options::addDoubleOption("auto-burst-start", 10.0);

    Options::addBooleanOption ("save_mesh", false);
    Options::addBooleanOption ("with_gravity", false);
    Options::addBooleanOption ("add_velocity", false);
    Options::addDoubleOption ("surface_tension", 1.0);
    
    Options::addStringOption ("sub_scene", "");
    
    Options::addBooleanOption ("two_side", false);
    Options::addBooleanOption ("advect", false);

    Options::addBooleanOption ("pulling", false);

    Options::parseOptionFile(option_file, m_verbose);

    // select the scene
    m_scene = Options::strValue("scene");

    std::vector<LosTopos::Vec3d> vertices;
    std::vector<LosTopos::Vec3st> faces;
    std::vector<LosTopos::Vec2i> face_labels;
    std::vector<size_t> constrained_vertices;
    std::vector<Vec3d> constrained_positions;
    
    
    if (m_scene == "inputmesh" or m_scene=="inputrec"){
    
        hgf = SZHScenes::sceneInputData(this, vertices, faces, face_labels, constrained_vertices, constrained_positions,inputdata_dir);
    }
    std::cout << "initial_nv = " << vertices.size() << ", initial_nf = " << faces.size() << std::endl;
    
    m_time = 0;
    m_dt = Options::doubleValue("time-step");

    m_finished = false;
    
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Time stepping
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sim::step()
{
    
    assert(m_scene != "unspecified");
    assert(hgf);
    assert(hgf->surfTrack());
    if (m_verbose)
        std::cout << "Time stepping: t = " << m_time << ", dt = " << m_dt << std::endl;
    

    static bool advect=Options::boolValue("advect");

    if(step_number %95 ==0){
        SZHScenes::burstBubbles(m_dt, this, hgf);
    }
    SZHScenes::volume_change(m_dt, this, hgf);
    
    double dt;
    static double computational_time=0.0;
    static double average_computational_time=0.0;
    static int count=0;
    const int count_offset=1;

    //For a naive adaptive time step.
    const bool time_adjust=false;
    if(time_adjust){
        static bool save_mesh_on=hgf->save_mesh;
        //if(true){
        if(count%4==0 ){
            //if(count%8<5 ){
            m_dt=0.01;
            if(save_mesh_on){
                hgf->save_mesh=true;
            }
        }else{
            m_dt=0.001;
            if(save_mesh_on){
                hgf->save_mesh=false;
            }
        }
    }

    if(hgf->logging_time ){

        if(count>=count_offset){
            // general time stepping
            static std::chrono::system_clock::time_point  start, end;
            
            start = std::chrono::system_clock::now();
            dt = hgf->step(m_dt);
            end = std::chrono::system_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //Convert time to ms.
            
            computational_time+=elapsed;
            average_computational_time=computational_time/(count-count_offset+1);
        }else{
            dt = hgf->step(m_dt);
        }
        
    }
    else{
        
        dt= hgf->step(m_dt);
    }

    // advance time
    m_frameid++;
    
    m_time+=dt;
    
    if (m_time >= Options::doubleValue("simulation-time"))
        m_finished = true;
    if(hgf->save_mesh){
        
        const double record_interval=0.01;
        const double eps=1e-6;
        static double passed_time=0;
        passed_time+=dt;
        
        if(passed_time>record_interval-eps){
            passed_time=0;
            
            //hgf->saveResult();
        }
        
    }
    step_number++;
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Rendering
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace
{
    bool is_edge_nonmanifold(const LosTopos::SurfTrack & st, size_t e)
    {
        return st.m_mesh.m_edge_to_triangle_map[e].size() != 2;
    }
    
    bool is_vertex_nonmanifold(const LosTopos::SurfTrack & st, size_t v)
    {
        for (size_t i = 0; i < st.m_mesh.m_vertex_to_edge_map[v].size(); i++)
            if (is_edge_nonmanifold(st, st.m_mesh.m_vertex_to_edge_map[v][i]))
                return true;
        return false;
    }
    
    bool is_face_next_to_nonmanifold_vertices(const LosTopos::SurfTrack & st, size_t f)
    {
        return is_vertex_nonmanifold(st, st.m_mesh.m_tris[f][0]) || is_vertex_nonmanifold(st, st.m_mesh.m_tris[f][1]) || is_vertex_nonmanifold(st, st.m_mesh.m_tris[f][2]);
    }
    
    bool is_edge_next_to_nonmanifold_vertices(const LosTopos::SurfTrack & st, size_t e)
    {
        return is_vertex_nonmanifold(st, st.m_mesh.m_edges[e][0]) || is_vertex_nonmanifold(st, st.m_mesh.m_edges[e][1]);
    }
    
    bool is_vertex_next_to_nonmanifold_vertices(const LosTopos::SurfTrack & st, size_t v)
    {
        for (size_t i = 0; i < st.m_mesh.m_vertex_to_edge_map[v].size(); i++)
            if (is_edge_next_to_nonmanifold_vertices(st, st.m_mesh.m_vertex_to_edge_map[v][i]))
                return true;
        return false;
    }
}