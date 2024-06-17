//
//  Sim.cpp
//
//  Fang Da 2014
//
//  Editted by Sadashige Ishida 2017

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
#include "YImage.h"
#include "Colormap.h"
#include "PRRenderer.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

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
m_nearest_face(-1),
m_prrenderer(NULL)
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
bool Sim::init(const std::string & option_file, bool save_outputs, bool wo_visualization,const std::string env_map_path,const std::string inputdata_dir)
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
    
        Options::addBooleanOption ("logging_geometry", true);
        Options::addBooleanOption ("write_geometry", true);
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
    
    Options::addBooleanOption("auto-burst", false);
    Options::addDoubleOption("auto-burst-interval", 20.0);
    Options::addDoubleOption("auto-burst-start", 10.0);

    Options::addBooleanOption ("save_mesh", false);
    Options::addBooleanOption ("with_gravity", false);
    Options::addBooleanOption ("add_velocity", false);
    Options::addBooleanOption ("accel", true);
    Options::addDoubleOption ("surface_tension", 1.0);
    
    Options::addStringOption ("sub_scene", "");
    
    Options::addBooleanOption ("two_side", false);
    Options::addBooleanOption ("advect", false);

    Options::addBooleanOption ("pulling", false);

    Options::parseOptionFile(option_file, m_verbose);

    // select the scene
    m_scene = Options::strValue("scene");

    if (save_outputs)
    {
        std::stringstream output_dir_ss;
        output_dir_ss << "output_" << ::time(NULL);
        m_output_directory = output_dir_ss.str();
        mkdir(m_output_directory.c_str(), 0755);
        std::cout << "Outputing to directory: " << m_output_directory << std::endl;
    }

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
    
    if(hgf->save_mesh){
        hgf->write_mesh();
        //hgf->writeObj_FaceLabel_constrainedVertices();
    }
    
    // output the initial frame
    if (m_output_directory != "" && Options::boolValue("output-mesh"))
        MeshIO::save(*hgf, m_output_directory + "/mesh000000.rec");

    // PR rendering
    if (!wo_visualization)
        m_prrenderer = new PRRenderer(hgf,env_map_path);

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
    if(hgf->bursting or Options::boolValue("auto-burst")){
        SZHScenes::burstBubbles                (m_dt, this, hgf);
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
            
            hgf->write_mesh();
            //hgf->writeObj_FaceLabel_constrainedVertices();
        }
        
    }
    
}

void Sim::stepOutput(bool wo_visualization)
{

    if (m_output_directory != "")
    {
        
        int frameid = (int)(time() / dt() + 0.5);

        int pngfd = Options::intValue("output-png-every-n-frames");
        if ((pngfd == 0 || frameid % pngfd == 0) && Options::boolValue("output-png") && !wo_visualization)
        {
            static int image_count=0;

            std::stringstream png_ss;
            
            png_ss << m_output_directory << "/frame" << std::setfill('0') << std::setw(6) << ++image_count << ".png";
            
            int w, h;
            w = glutGet(GLUT_WINDOW_WIDTH);
            h = glutGet(GLUT_WINDOW_HEIGHT);
            
            YImage img;
            img.resize(w, h);
            glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char *)(img.data()));
            img.flip();
            img.save(png_ss.str().c_str());
        }

            int meshfd = Options::intValue("output-mesh-every-n-frames");
            if ((meshfd == 0 || frameid % meshfd == 0) && Options::boolValue("output-mesh"))
            {
                static int rec_count=0;

                std::stringstream mesh_ss;
                mesh_ss << m_output_directory << "/state" << std::setfill('0') << std::setw(6) << rec_count++ << ".rec";
                MeshIO::save(*hgf, mesh_ss.str());
            }
            
            int objfd = Options::intValue("output-obj-every-n-frames");
            if ((objfd == 0 || frameid % objfd == 0) && Options::boolValue("output-obj"))
            {
                static int obj_count=0;
                
                std::stringstream obj_ss;
                obj_ss << m_output_directory << "/mesh" << std::setfill('0') << std::setw(6) << obj_count++ << ".obj";
                MeshIO::saveOBJ(*hgf, obj_ss.str());
            }

    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Loading saved simulation
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Sim::load(int inc)
{
    int current_frame = (int)((m_time + m_dt * 0.5) / m_dt);
    int next_frame = current_frame + inc;
    if (next_frame < 0) next_frame = 0;
    
    std::stringstream ss;
    ss << m_load_directory << "/mesh" << std::setfill('0') << std::setw(6) << next_frame << ".rec";
    if (!MeshIO::load(*hgf, ss.str()))
    {
        std::cout << "Loading frame " << ss.str() << " unsuccessful." << std::endl;
        return false;
    }
    
    m_time = m_dt * next_frame;
    
    return true;
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

void Sim::render(RenderMode rm, const Vec2d & mousepos)
{
    if (rm == RM_PR)
    {
        if(m_prrenderer->env_map_path==""){
            return;
        }
        m_prrenderer->render();
        return;
    }
}