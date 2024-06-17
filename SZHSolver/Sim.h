//
//  Sim.h
//
//  Fang Da 2014

#ifndef __Sim__
#define __Sim__

#include <iostream>
#include <string>
#include "SZHSolver.h"
#include "Scenes.h"

class Sim
{
    friend class Scenes;
public:
    Sim(bool verbose);
    ~Sim();
    
    SZHSolver * get_hgf() { return hgf; }
    
public:
    bool init(const std::string & option_file,const std::string inputdata_dir="");
    
public:
    void step();
    bool isFinished() const { return m_finished; }
    
    double dt() const { return m_dt; }
    double time() const { return m_time; }
    
public:
    bool camera_information;

public:
    bool m_verbose;
    int step_number =0;
    std::string m_scene;
    std::string m_output_directory;
    std::string m_load_directory;
    
    SZHSolver * hgf;
    double m_dt;
    double m_time;
    int m_frameid;
    bool m_finished;

    int m_nearest_vertex;
    int m_nearest_edge;
    int m_nearest_face;
  
};

#endif
