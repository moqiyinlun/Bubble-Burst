
#include "vec.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <stdio.h>
#include <map>
#include <numeric>

#include <Eigen/Sparse>
#include <igl/per_vertex_normals.h>
#include <igl/writeOBJ.h>
#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/volume.h>
#include <igl/massmatrix.h>
#include "MeshIO.h"
#include "Options.h"
#include "SZHSolver.h"
#include "Options.h"
#include "LosTopos/LosTopos3D/subdivisionscheme.h"

void SZHSolver::set_parameters(){
    
    delta_dn_tq_juctions=1;

    accel=Options::boolValue("accel");
    damp= Options::doubleValue("damping-coef");//1.0;0.999 ;0.995;1-dt*damp_scale.
    surface_tension=Options::doubleValue("surface_tension");;

    smooth=Options::doubleValue("smoothing-coef");
    const double eps=0.0001;
    save_mesh=Options::boolValue("save_mesh");
    sparse=Options::boolValue("sparse");
    
    //Used for logging information.
    logging_geometry=Options::boolValue("logging_geometry");
    write_geometry=Options::boolValue("write_geometry");;
    logging_time=Options::boolValue("logging_time");;
    logging_detailed_time=Options::boolValue("logging_detailed_time");;
    
    if(logging_detailed_time){
        time_dAdx=0.0;
        time_volume_correction=0.0;
        time_los_topos=0.0;
    }

    with_gravity=Options::boolValue("with_gravity");
    add_velocity=Options::boolValue("add_velocity");
    double gravity_scale= 0.02;//0.007;
    gravity_vec<<0,0,-9.8*gravity_scale;

    energy_preserve_per_vertex=0;
    energy_preserve_with_max_velocity=false;
    
    local_energy_preservation=0;

    blowing_bubble0=false;
    
    bursting=false;
    
    do_stepScenes=false;

}

SZHSolver::SZHSolver(const std::vector<LosTopos::Vec3d> & vs, const std::vector<LosTopos::Vec3st> & fs, const std::vector<LosTopos::Vec2i> & ls, const std::vector<size_t> & constrained_vertices,  const std::vector<Vec3d> & constrained_positions, const int _num_bubbles):num_bubbles(_num_bubbles)
{

    // construct the surface tracker
    double mean_edge_len = Options::doubleValue("remeshing-resolution");
    if (mean_edge_len == 0)
    {
        for (size_t i = 0; i < fs.size(); i++)
        {
            mean_edge_len += mag(vs[fs[i][0]] - vs[fs[i][1]]);
            mean_edge_len += mag(vs[fs[i][1]] - vs[fs[i][2]]);
            mean_edge_len += mag(vs[fs[i][2]] - vs[fs[i][0]]);
        }
        mean_edge_len /= (fs.size() * 3);
    }
    double min_edge_len = mean_edge_len * 0.5;
    double max_edge_len = mean_edge_len * 1.5;
    LosTopos::SurfTrackInitializationParameters params;
    params.m_proximity_epsilon = Options::doubleValue("lostopos-collision-epsilon-fraction") * mean_edge_len;
    params.m_merge_proximity_epsilon = Options::doubleValue("lostopos-merge-proximity-epsilon-fraction") * mean_edge_len;
    params.m_allow_vertex_movement_during_collapse = true;
    params.m_perform_smoothing = Options::boolValue("lostopos-perform-smoothing");
    params.m_min_edge_length = min_edge_len;
    params.m_max_edge_length = max_edge_len;
    params.m_max_volume_change = Options::doubleValue("lostopos-max-volume-change-fraction") * pow(mean_edge_len, 3);
    params.m_min_triangle_angle = Options::doubleValue("lostopos-min-triangle-angle");
    params.m_max_triangle_angle = Options::doubleValue("lostopos-max-triangle-angle");
    params.m_large_triangle_angle_to_split = Options::doubleValue("lostopos-large-triangle-angle-to-split");
    params.m_min_triangle_area = Options::doubleValue("lostopos-min-triangle-area-fraction") * pow(mean_edge_len, 2);
    params.m_verbose = false;
    params.m_allow_non_manifold = Options::boolValue("lostopos-allow-non-manifold");
    params.m_allow_topology_changes = Options::boolValue("lostopos-allow-topology-changes");
    params.m_collision_safety = true;
    params.m_remesh_boundaries = true;
    params.m_t1_transition_enabled = Options::boolValue("lostopos-t1-transition-enabled");
    params.m_pull_apart_distance = Options::doubleValue("lostopos-t1-pull-apart-distance-fraction") * mean_edge_len;
    
    params.m_velocity_field_callback = NULL;
    
    if (Options::boolValue("lostopos-smooth-subdivision"))
        params.m_subdivision_scheme = new LosTopos::ModifiedButterflyScheme();
    else
        params.m_subdivision_scheme = new LosTopos::MidpointScheme();
    
    params.m_use_curvature_when_collapsing = false;
    params.m_use_curvature_when_splitting = false;

    m_constrained_vertices = constrained_vertices;
    m_constrained_positions = constrained_positions;
    
    std::vector<LosTopos::Vec3d> masses(vs.size(), LosTopos::Vec3d(1, 1, 1));
    for (size_t i = 0; i < m_constrained_vertices.size(); i++)
        masses[m_constrained_vertices[i]] *= std::numeric_limits<double>::infinity();
    m_st = new LosTopos::SurfTrack(vs, fs, ls, masses, params);
        
    m_st->m_solid_vertices_callback = this;
    m_st->m_mesheventcallback = this;

    // find out the number of regions
    num_region = 0;
    for (size_t i = 0; i < mesh().nt(); i++)
    {
        LosTopos::Vec2i l = mesh().get_triangle_label(i);
        num_region = std::max(num_region, l[0] + 1);
        num_region = std::max(num_region, l[1] + 1);
    }
    // initialize the dynamicc quantities i.e. velocity.
    m_v = new LosTopos::NonDestructiveTriMesh::VertexData<Vec3d>(&(m_st->m_mesh));
    for (size_t i = 0; i < mesh().nv(); i++){
        (*m_v)[i] =Vec3d(0,0,0);
    }
    //compute initial volumes
    Eigen::VectorXd volumes;
    Eigen::MatrixXd area_matrix;

    if(num_bubbles==-1){
        num_bubbles=num_region-1;
        AIR=0;
    }
    
    region_offset=num_region-num_bubbles;
    
    AIR=region_offset-1;
    set_parameters();
    
    //Necessary for volume correction.
    easy_orientation();

    if(num_bubbles>0){
        volumes_and_areas( volumes, area_matrix);
    }
    initialVolumes=volumes;
    if(logging_geometry){
        using std::cout;using std::endl;
        cout<<"initial volumes:";
        for(int bi=0;bi<num_bubbles;++bi){
            cout<<initialVolumes[bi]<<" ";
        }
        cout<<endl;
        
        cout<<"initial areas:";
        for(int bi=0;bi<num_bubbles;++bi){
            cout<<area_matrix(bi,bi)<<" ";
        }
        cout<<endl;
    }
}

SZHSolver::~SZHSolver()
{
    if (m_st)
        delete m_st;
}

double SZHSolver::step(double dt){
    
    actual_dt=dt;
    
    static int count=0;
    ++count;
    static std::chrono::system_clock::time_point  start, mid1, mid2,end;
    
    auto nv=mesh().nv();

    {
        
        double old_energy;

//        nv=mesh().nv();
//        for (size_t i = 0; i < nv; i++){
//            m_st->pm_velocities[i] = vc(vel(i));
//        }
        
        auto los_topos_part=[this](){
            // Mesh improvement
            for(int i = 0; i < Options::intValue("remeshing-iterations"); i++)
            {
                
                m_st->topology_changes();
                m_st->improve_mesh();
                
                //Necessary for volume correction.
                easy_orientation();
                
            }
        };

        if(logging_detailed_time and count>1){
            start = std::chrono::system_clock::now();
            
            los_topos_part();
            
            mid1 = std::chrono::system_clock::now();

        }else{
            los_topos_part();
        }
        
        

        // defrag the mesh in the end, to ensure the next step starts with a clean mesh
        m_st->defrag_mesh_from_scratch(m_constrained_vertices);

    }

    // damping by smoothing

    
    //Enforce constrained vertices to be fixed.
    if(1)
    {
        // before enforcing constraints, first scan through the mesh to find any solid vertices not registered as constraints. they can appear due to remeshing (splitting an all-solid edge)
        std::vector<int> constrained_vertices_map(mesh().nv(), -1);
        for (size_t i = 0, n_consv=m_constrained_vertices.size(); i < n_consv; i++)
            constrained_vertices_map[m_constrained_vertices[i]] = i;
        
        for (size_t i = 0,nv=mesh().nv(); i <nv; i++)
        {
            if (m_st->vertex_is_any_solid(i) && constrained_vertices_map[i] < 0)
            {
                m_constrained_vertices.push_back(i);
                m_constrained_positions.push_back(pos(i));
            }
        }

        //    // enforce the constraints exactly.
        for (size_t ii = 0,n_consv=m_constrained_vertices.size(); ii <n_consv ; ii++)
        {
            size_t i = m_constrained_vertices[ii];
            m_st->pm_newpositions[i] = vc(m_constrained_positions[ii]);
            vel(i)<<0,0,0;
        }
        
    }
    stepHGF(dt);
    // move the mesh
    auto integrate_part=[this,dt](){
        double actual_dt;
        m_st->integrate(dt, actual_dt);
        if (actual_dt != dt)
            std::cout << "Warning: SurfTrack::integrate() failed to step the full length of the time step!" << std::endl;

    };
    
    if(logging_detailed_time and count>1){
        
        mid2=std::chrono::system_clock::now();
        integrate_part();
        end = std::chrono::system_clock::now();

        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>((end-mid2)+(mid1-start)).count(); //Convert time to ms.

        time_los_topos+=elapsed;
        
    }else{
        integrate_part();
    }

    return actual_dt;
    
}

//
//Below are the functions for LosTopos, including the interpolation of the velocity for a newly genereted vertex.
//

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Callbacks
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SZHSolver::generate_collapsed_position(LosTopos::SurfTrack & st, size_t v0, size_t v1, LosTopos::Vec3d & pos)
{
    if (st.vertex_is_any_solid(v0) && st.vertex_is_any_solid(v1))
    {
        return false;
    } else if (st.vertex_is_any_solid(v0))
    {
        pos = st.pm_positions[v0];
        return true;
    } else if (st.vertex_is_any_solid(v1))
    {
        pos = st.pm_positions[v1];
        return true;
    } else
    {
        pos = (st.pm_positions[v0] + st.pm_positions[v1]) / 2;
        return true;
    }
}

bool SZHSolver::generate_split_position(LosTopos::SurfTrack & st, size_t v0, size_t v1, LosTopos::Vec3d & pos)
{
    std::cout << "solid callback: generate split position: " << v0 << " " << v1 << " " << (st.vertex_is_any_solid(v0) && st.vertex_is_any_solid(v1)) << std::endl;
    pos = (st.pm_positions[v0] + st.pm_positions[v1]) / 2;
    if (st.vertex_is_any_solid(v0) && st.vertex_is_any_solid(v1))
        return false;
    else
        return true;
}


LosTopos::Vec3c SZHSolver::generate_split_solid_label(LosTopos::SurfTrack & st, size_t v0, size_t v1, const LosTopos::Vec3c & label0, const LosTopos::Vec3c & label1)
{
    if (st.vertex_is_any_solid(v0) && st.vertex_is_any_solid(v1))
        return LosTopos::Vec3c(1, 1, 1);
    else if (st.vertex_is_any_solid(v0))
        return LosTopos::Vec3c(0, 0, 0);
    else if (st.vertex_is_any_solid(v1))
        return LosTopos::Vec3c(0, 0, 0);
    else
        return LosTopos::Vec3c(0, 0, 0);
}



LosTopos::Vec3d SZHSolver::sampleVelocity(LosTopos::Vec3d & pos)
{
    return LosTopos::Vec3d(0, 0, 0);
}

struct CollapseTempData
{
    size_t v0;
    size_t v1;
    
    Vec3d old_x0;
    Vec3d old_x1;
    
    Vec3d old_u0;
    Vec3d old_u1;
    
    //For local energy preservation
    double old_local_energy;
    double local_area;
};

void SZHSolver::pre_collapse(const LosTopos::SurfTrack & st, size_t e, void ** data)
{
    CollapseTempData * td = new CollapseTempData;
    td->v0 = st.m_mesh.m_edges[e][0];
    td->v1 = st.m_mesh.m_edges[e][1];
    
    td->old_x0 = vc(st.pm_positions[td->v0]);
    td->old_x1 = vc(st.pm_positions[td->v1]);
    
    td->old_u0 = vel(td->v0);
    td->old_u1 = vel(td->v1);

    *data = (void *)td;
    std::cout << "pre collapse: " << e << ": " << td->v0 << " " << td->v1 << std::endl;

    if(local_energy_preservation){
        
        auto target_faces0=st.m_mesh.m_vertex_to_triangle_map [td->v0];
        auto target_faces1=st.m_mesh.m_vertex_to_triangle_map [td->v1];
        
        std::vector<size_t> target_faces=target_faces0;
        
        for(auto face1:target_faces1){
            bool already_in=std::find(target_faces.begin(),target_faces.end(),face1)!=target_faces.end();
            
            if(!already_in){
                target_faces.push_back(face1);
            }
        }

        double local_energy=0;
        
        double local_area=0;
        
        for(auto face:target_faces){
            LosTopos::Vec3st tri=st.m_mesh.m_tris[face];
            
            double tri_area=face_area(face);
            local_area+=tri_area;
            
            for(size_t vi=0;vi<3;++vi){
                size_t v=tri[vi];
                Vec3d velocity=vel(v);
                local_energy+=1.0/3.0*tri_area*velocity.squaredNorm();
            }
        }
        
        td->old_local_energy=local_energy;
        
        td->local_area=local_area;

    }
}

void SZHSolver::post_collapse(const LosTopos::SurfTrack & st, size_t e, size_t merged_vertex, void * data)
{
    CollapseTempData * td = (CollapseTempData *)data;
    std::cout << "post collapse: " << e << ": " << td->v0 << " " << td->v1 << " => " << merged_vertex << std::endl;
    assert((st.m_mesh.vertex_is_deleted(td->v0) && merged_vertex == td->v1) || (st.m_mesh.vertex_is_deleted(td->v1) && merged_vertex == td->v0));
    
    Vec3d merged_x = vc(st.pm_positions[merged_vertex]);
    double s = (merged_x - td->old_x0).dot(td->old_x1 - td->old_x0) / (td->old_x1 - td->old_x0).squaredNorm();

    if(true){
        if (s > 1) s = 1;
        if (s < 0) s = 0;
        
        Vec3d new_u= td->old_u0 * (1 - s) + td->old_u1 * s;

        if(energy_preserve_per_vertex){
            double new_norm=(1-s)*td->old_u0.norm()+s*td->old_u1.norm();
            
            if(energy_preserve_with_max_velocity){
                new_norm=std::max(td->old_u0.norm(),td->old_u1.norm());
            }
            new_u=new_norm*new_u.normalized();
            
        }
        
        vel(merged_vertex) = new_u;
    }
    
    if(local_energy_preservation){
        auto target_faces=st.m_mesh.m_vertex_to_triangle_map [merged_vertex];

        double new_local_energy=0;
        double vertex_area=0;
        
        double local_area=0;
        
        for(auto face:target_faces){
            LosTopos::Vec3st tri=st.m_mesh.m_tris[face];
            
            double tri_area=face_area(face);
            vertex_area+=1.0/3.0*tri_area;
            
            local_area+=tri_area;
            
            for(size_t vi=0;vi<3;++vi){
                size_t v=tri[vi];
                
                if(v==merged_vertex){continue;}
                Vec3d velocity=vel(v);
                new_local_energy+=1.0/3.0*tri_area*velocity.squaredNorm();
            }
        }
        
        double new_squared_norm=(td->old_local_energy-new_local_energy)/vertex_area;

        vel(merged_vertex)=vel(merged_vertex).normalized()*sqrt(new_squared_norm);

    }

    //Smmooth the velocity of the new vertex.
    if(false){
        
        Vec3d newVelocity(0,0,0);
        
        newVelocity=Vec3d(0,0,0);
        
        Vec3d neighborhood_mean_velocity(0,0,0);
        
        int neighborhood_counter = 0;
        for (size_t k = 0; k < st.m_mesh.m_vertex_to_edge_map[merged_vertex].size(); k++)
        {
            LosTopos::Vec2st e = st.m_mesh.m_edges[st.m_mesh.m_vertex_to_edge_map[merged_vertex][k]];
            size_t vother = (e[0] == merged_vertex ? e[1] : e[0]);

            neighborhood_mean_velocity += vel(vother);
            neighborhood_counter++;
            
        }
        if (neighborhood_counter != 0){
            neighborhood_mean_velocity/=neighborhood_counter;
            
        }
        
        double smoothing_coef=0.5;//0 ~ 1.
        
        newVelocity=vel(merged_vertex)+(neighborhood_mean_velocity-vel(merged_vertex))* smoothing_coef;

        vel(merged_vertex) = newVelocity;
        
    }
    
}

struct SplitTempData
{
    size_t v0;
    size_t v1;
    
    Vec3d old_x0;
    Vec3d old_x1;
    
    Vec3d old_u0;
    Vec3d old_u1;

    //For local energy preservation
    double old_local_energy;
    double local_area;
};

void SZHSolver::pre_split(const LosTopos::SurfTrack & st, size_t e, void ** data)
{
    SplitTempData * td = new SplitTempData;
    td->v0 = st.m_mesh.m_edges[e][0];
    td->v1 = st.m_mesh.m_edges[e][1];
    
    td->old_x0 = vc(st.pm_positions[td->v0]);
    td->old_x1 = vc(st.pm_positions[td->v1]);
    
    td->old_u0 = vel(td->v0);
    td->old_u1 = vel(td->v1);

    *data = (void *)td;
    std::cout << "pre split: " << e << ": " << td->v0 << " " << td->v1 << std::endl;
    
    if(local_energy_preservation){
        auto edges=st.m_mesh.m_vertex_to_edge_map[td->v0];
        size_t common_edge=-1;
        for(auto edge:edges){
            size_t vother=edge_other_vertex(edge, td->v0);
            if(vother==td->v1){
                common_edge=edge;
                break;
            }
        }
        assert(common_edge!=-1);
        
        auto target_faces=st.m_mesh.m_edge_to_triangle_map[common_edge];
        
        assert(target_faces.size()==2 );
        
        double local_energy=0;
        double local_area=0;
        
        for(auto face:target_faces){
            LosTopos::Vec3st tri=st.m_mesh.m_tris[face];
            
            double tri_area=face_area(face);
            local_area+=tri_area;
            
            for(size_t vi=0;vi<3;++vi){
                size_t v=tri[vi];
                Vec3d velocity=vel(v);
                local_energy+=1.0/3.0*tri_area*velocity.squaredNorm();
            }
        }
        
        td->old_local_energy=local_energy;
        
        td->local_area=local_area;
        
    }

}

void SZHSolver::post_split(const LosTopos::SurfTrack & st, size_t e, size_t new_vertex, void * data)
{
    SplitTempData * td = (SplitTempData *)data;
    std::cout << "post split: " << e << ": " << td->v0 << " " << td->v1 << " => " << new_vertex << std::endl;
    
    Vec3d midpoint_x = vc(st.pm_positions[new_vertex]);
    double s = (midpoint_x - td->old_x0).dot(td->old_x1 - td->old_x0) / (td->old_x1 - td->old_x0).squaredNorm();

    if(true){
        if (s > 1) s = 1;
        if (s < 0) s = 0;
        
        Vec3d new_u= td->old_u0 * (1 - s) + td->old_u1 * s;

        if(energy_preserve_per_vertex){
            double new_norm=(1-s)*td->old_u0.norm()+s*td->old_u1.norm();
            
            if(energy_preserve_with_max_velocity){
                new_norm=std::max(td->old_u0.norm(),td->old_u1.norm());
            }
            new_u=new_norm*new_u.normalized();

        }
        
        vel(new_vertex) = new_u;
    }

    if(local_energy_preservation){
        auto target_faces=st.m_mesh.m_vertex_to_triangle_map [new_vertex];

        assert(target_faces.size()==4);
        double new_local_energy=0;
        double vertex_area=0;
        double local_area=0;
        
        for(auto face:target_faces){
            LosTopos::Vec3st tri=st.m_mesh.m_tris[face];
            
            double tri_area=face_area(face);
            vertex_area+=1.0/3.0*tri_area;
            
            local_area+=tri_area;
            
            for(size_t vi=0;vi<3;++vi){
                size_t v=tri[vi];
                
                if(v==new_vertex){continue;}
                Vec3d velocity=vel(v);
                new_local_energy+=1.0/3.0*tri_area*velocity.squaredNorm();
            }
        }
        
        auto v_area=vert_area(new_vertex);
        double new_squared_norm=(td->old_local_energy-new_local_energy)/vertex_area;

        vel(new_vertex)=vel(new_vertex).normalized()*sqrt(new_squared_norm);

    }
    
    //Smmooth the velocity o the new vertex.
    if(false){
        
        Vec3d newVelocity(0,0,0);
        
        newVelocity=Vec3d(0,0,0);
        
        Vec3d neighborhood_mean_velocity(0,0,0);
        
        int neighborhood_counter = 0;
        for (size_t k = 0; k < st.m_mesh.m_vertex_to_edge_map[new_vertex].size(); k++)
        {
            LosTopos::Vec2st e = st.m_mesh.m_edges[st.m_mesh.m_vertex_to_edge_map[new_vertex][k]];
            size_t vother = (e[0] == new_vertex ? e[1] : e[0]);

            neighborhood_mean_velocity += vel(vother);
            neighborhood_counter++;
            
        }
        if (neighborhood_counter != 0){
            neighborhood_mean_velocity/=neighborhood_counter;
            
        }
        
        double smoothing_coef=0.5;//0 ~ 1.
        
        newVelocity=vel(new_vertex)+(neighborhood_mean_velocity-vel(new_vertex))* smoothing_coef;

        vel(new_vertex) = newVelocity;
        
    }
    
}

void SZHSolver::pre_flip(const LosTopos::SurfTrack & st, size_t e, void ** data)
{
    // virtual function... void one 
}

void SZHSolver::post_flip(const LosTopos::SurfTrack & st, size_t e, void * data)
{
    // virtual function... void one 
}

struct T1TempData
{
    
};

void SZHSolver::pre_t1(const LosTopos::SurfTrack & st, size_t v, void ** data)
{
    T1TempData * td = new T1TempData;
    
    *data = (void *)td;
}

void SZHSolver::post_t1(const LosTopos::SurfTrack & st, size_t v, size_t a, size_t b, void * data)
{
    std::cout << "v = " << v << " -> " << a << " " << b << std::endl;
    T1TempData * td = (T1TempData *)data;

    if(true){
        vel(a) = vel(v);
        vel(b) = vel(v);
    }
}

struct FaceSplitTempData
{
    size_t v0;
    size_t v1;
    size_t v2;
    
    Vec3d old_x0;
    Vec3d old_x1;
    Vec3d old_x2;
    
    Vec3d old_u0;
    Vec3d old_u1;
    Vec3d old_u2;
};

void SZHSolver::pre_facesplit(const LosTopos::SurfTrack & st, size_t f, void ** data)
{
    FaceSplitTempData * td = new FaceSplitTempData;
    
    td->v0 = st.m_mesh.get_triangle(f)[0];
    td->v1 = st.m_mesh.get_triangle(f)[1];
    td->v2 = st.m_mesh.get_triangle(f)[2];

    td->old_x0 = vc(st.pm_positions[td->v0]);
    td->old_x1 = vc(st.pm_positions[td->v1]);
    td->old_x2 = vc(st.pm_positions[td->v2]);
    
    td->old_u0 = vel(td->v0);
    td->old_u1 = vel(td->v1);
    td->old_u2 = vel(td->v2);
    
    *data = (void *)td;
}

void SZHSolver::post_facesplit(const LosTopos::SurfTrack & st, size_t f, size_t new_vertex, void * data)
{
    FaceSplitTempData * td = (FaceSplitTempData *)data;

    if(true){
        Vec3d new_x = vc(st.pm_positions[new_vertex]);
        Vec3d c = Vec3d::Zero();
        Vec3d n = (td->old_x1 - td->old_x0).cross(td->old_x2 - td->old_x0);
        double nsq = n.squaredNorm();
        c[0] = 1 - (new_x - td->old_x0).dot(n.cross(td->old_x1 - td->old_x2)) / nsq;
        c[1] = 1 - (new_x - td->old_x1).dot(n.cross(td->old_x2 - td->old_x0)) / nsq;
        if (c[0] > 1)        c[0] = 1;
        if (c[0] < 0)        c[0] = 0;
        if (c[1] > 1 - c[0]) c[1] = 1 - c[0];
        if (c[1] < 0)        c[1] = 0;
        c[2] = 1 - c[0] - c[1];
        
        vel(new_vertex) = td->old_u0 * c[0] + td->old_u1 * c[1] + td->old_u2 * c[2];
        
        if(energy_preserve_per_vertex){
            double new_norm=c[0]*td->old_u0.norm()+c[1]*td->old_u1.norm()*c[2]*td->old_u2.norm();
            
            if(energy_preserve_with_max_velocity){
                std::vector<double> norms={td->old_u0.norm(),td->old_u1.norm(),td->old_u2.norm()};
                new_norm=*std::max_element(norms.begin(),norms.end());
            }
            
            vel(new_vertex)=new_norm*vel(new_vertex).normalized();
        }
        
    }
}

struct SnapTempData
{
    size_t v0;
    size_t v1;
    
    Vec3d old_x0;
    Vec3d old_x1;

    Vec3d old_u0;
    Vec3d old_u1;
};

void SZHSolver::pre_snap(const LosTopos::SurfTrack & st, size_t v0, size_t v1, void ** data)
{
    SnapTempData * td = new SnapTempData;
    td->v0 = v0;
    td->v1 = v1;
    
    td->old_x0 = vc(st.pm_positions[v0]);
    td->old_x1 = vc(st.pm_positions[v1]);

    td->old_u0 = vel(td->v0);
    td->old_u1 = vel(td->v1);

    *data = (void *)td;
    std::cout << "pre snap: " << v0 << " " << v1 << std::endl;
}

void SZHSolver::post_snap(const LosTopos::SurfTrack & st, size_t v_kept, size_t v_deleted, void * data)
{
    SnapTempData * td = (SnapTempData *)data;
    std::cout << "post snap: " << td->v0 << " " << td->v1 << " => " << v_kept << std::endl;
    assert((td->v0 == v_kept && td->v1 == v_deleted) || (td->v1 == v_kept && td->v0 == v_deleted));
    assert(v_kept != v_deleted);
    assert(st.m_mesh.vertex_is_deleted(v_deleted));
    assert(!st.m_mesh.vertex_is_deleted(v_kept));
    
    Vec3d merged_x = vc(st.pm_positions[v_kept]);
    double s = (merged_x - td->old_x0).dot(td->old_x1 - td->old_x0) / (td->old_x1 - td->old_x0).squaredNorm();

    if(true){
        
        if (s > 1) s = 1;
        if (s < 0) s = 0;
        Vec3d new_u = td->old_u0 * (1 - s) + td->old_u1 * s;

        if(energy_preserve_per_vertex){
            double new_norm=(1-s)*td->old_u0.norm()+s*td->old_u1.norm();

            if(energy_preserve_with_max_velocity){
                new_norm=std::max(td->old_u0.norm(),td->old_u1.norm());
                
            }
            new_u=new_norm*new_u.normalized();
            
        }
        
        vel(v_kept) = new_u;
    }
    
    //Smmooth the velocity of the new vertex.
    if(false){
        Vec3d newVelocity(0,0,0);
        newVelocity=Vec3d(0,0,0);
        
        Vec3d neighborhood_mean_velocity(0,0,0);
        
        int neighborhood_counter = 0;
        for (size_t k = 0; k < st.m_mesh.m_vertex_to_edge_map[v_kept].size(); k++)
        {
            LosTopos::Vec2st e = st.m_mesh.m_edges[st.m_mesh.m_vertex_to_edge_map[v_kept][k]];
            size_t vother = (e[0] == v_kept ? e[1] : e[0]);

            neighborhood_mean_velocity += vel(vother);
            neighborhood_counter++;
            
        }
        if (neighborhood_counter != 0){
            neighborhood_mean_velocity/=neighborhood_counter;
            
        }
        
        double smoothing_coef=0.5;//0 ~ 1.
        
        newVelocity=vel(v_kept)+(neighborhood_mean_velocity-vel(v_kept))* smoothing_coef;

        vel(v_kept) = newVelocity;
        
    }
    
}

void SZHSolver::set_intermediate_implicit(double dt,Eigen::MatrixXd &U,Eigen::MatrixXi& F,Eigen::MatrixXd &dUdt,Eigen::MatrixXd &NewU){
    using namespace Eigen;
    size_t nv=U.rows();
    size_t nt=F.rows();
    
    Eigen::SparseMatrix<double>  L;//cotMatrix
    igl::cotmatrix(U,F,L);

    // Recompute just mass matrix on each step
    SparseMatrix<double> M;
    igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_BARYCENTRIC,M);

    const auto & S = (M - dt*L);

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);

    if(accel){
        
        NewU=solver.solve(M*U).eval();

        MatrixXd D2UDt2=surface_tension*(NewU-U)/dt;
        if(with_gravity){
            for(int vi=0;vi<nv;++vi){
                D2UDt2.row(vi)+=gravity_vec;
            }
        }
        
        dUdt+=D2UDt2*dt;
        NewU=U+dUdt*dt;

    }else{
        NewU = solver.solve(M*U).eval();
        
    }
    
}

void SZHSolver::set_intermediate_symplectic_euler(double dt,Eigen::MatrixXd &U,Eigen::MatrixXi& F,Eigen::MatrixXd &dUdt,Eigen::MatrixXd &NewU){
    using namespace Eigen;
    size_t nv=U.rows();
    size_t nt=F.rows();
    
    MatrixXd dAdx;
    
    Eigen::SparseMatrix<double>  L;//cotMatrix
    
    igl::cotmatrix(U,F,L);
    
    SparseMatrix<double> M,Minv;

    igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_BARYCENTRIC,M);
    igl::invert_diag(M,Minv);
    // Laplace-Beltrami of position
    dAdx = -Minv*(L*U);
    
    if(accel){
        MatrixXd D2UDt2=-surface_tension*dAdx;
        
        if(with_gravity){
            //static Vector3d gravity_vector(0,0,-0.098);
            for(int vi=0;vi<nv;++vi){
                D2UDt2.row(vi)+=gravity_vec;
            }
        }

        dUdt+=D2UDt2*dt;
        NewU=U+dUdt*dt;
        
        //For adaptive time step, according to the maximal curvature.
        //Not available in the current implementation.
        //Extract mgnitude as mean curvature
        //
        //VectorXd H = dAdx.rowwise().norm();
        //double coef=1.;//0.1;
        //double ddt=coef/H.maxCoeff();
        //dUdt+=D2UDt2*ddt;
        //NewU=U+dUdt*ddt;
        
    }else{
        double small_scale=1.0;
        auto DU=-small_scale*surface_tension*dAdx*dt;
        NewU=U+DU;
    }
    
}

void SZHSolver::stepHGF(double dt){

    using namespace Eigen;
    
    size_t nv=m_st->m_mesh.nv();
    size_t nt=m_st->m_mesh.nt();
    MatrixXd U=MatrixXd::Zero(nv, 3);
    MatrixXi F=MatrixXi::Zero(nt, 3);
    MatrixXd dUdt=MatrixXd::Zero(nv, 3);
    MatrixXd NewU;
    
    //Convert vertices and faces to U,F, duDt.
    for (size_t i = 0; i < nv; i++)
    {
        U.row(i)<<m_st->pm_positions[i][0],m_st->pm_positions[i][1],m_st->pm_positions[i][2];

        //Do not use m_st->pm_velocities.
        //It's not defraged by LosTopos.
        //Instead use "vel" function.
        dUdt.row(i)=vel(i);
        
    }
    for (size_t i = 0; i < nt; i++)
    {
        F.row(i)<<m_st->m_mesh.m_tris[i][0],m_st->m_mesh.m_tris[i][1],m_st->m_mesh.m_tris[i][2];
    }

    auto update=[this,&U,&F,&dUdt,&NewU,dt](){
        set_intermediate_symplectic_euler(dt, U, F, dUdt,NewU);
    };
    
    static int count=0;
    ++count;
    if(logging_detailed_time and count>1){
        
        // general time stepping
        std::chrono::system_clock::time_point  start, end;
        start = std::chrono::system_clock::now();
        
        update();
        
        end = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //Convert time to ms.
        
        time_dAdx+=elapsed;

    }else{
        
        update();

    }

    //Ensure constrained vertices to be fixed.
    //Maybe not necessary.
    int nv_cons=constrainedVertices().size();
    for(int cvi=0;cvi<nv_cons;++cvi){
        int cv=m_constrained_vertices[cvi];
        NewU.row(cv)=m_constrained_positions[cvi];
        vel(cv)<<0,0,0;
    }

    if(logging_detailed_time and count>1){

        // general time stepping
        std::chrono::system_clock::time_point  start, end;
        start = std::chrono::system_clock::now();
        
        correct_volume(NewU, F);
        
        end = std::chrono::system_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //Convert time to ms.
        
        time_volume_correction+=elapsed;

    }else{
        
        correct_volume(NewU, F);

    }

    dUdt=(NewU-U)/dt;
    
    dUdt*=damp;
    
    //Convert U and dUdt to vertices and faces.
    for (size_t i = 0; i < nv; i++)
    {
        
        m_st->pm_newpositions[i]=vc(NewU.row(i));
        //Do not use m_st->pm_velocities.
        //It's not defraged by LosTopos.
        //Instead use "vel" function.
        vel(i)=dUdt.row(i);
        
    }

    for(int cvi=0;cvi<nv_cons;++cvi){
        int cv=m_constrained_vertices[cvi];
        m_st->pm_newpositions[cv]=vc(m_constrained_positions[cvi]);
        m_st->pm_positions[cv]=vc(m_constrained_positions[cvi]);
        
        vel(cv)<<0,0,0;

    }

}

//Correct volume of each closed region.
//This is an extension of [Muller 2009]"Fast and Robust Tracking of Fluid Surfaces" for multiple regions.
void  SZHSolver::correct_volume(Eigen::MatrixXd &targetU,Eigen::MatrixXi& F){
    using namespace Eigen;
    
    MatrixXd Delta_d_n;
    computeDelta_d_n(targetU,F, Delta_d_n);
    targetU+=Delta_d_n;
    
}

void SZHSolver::computeDelta_d_n(const Eigen::MatrixXd &targetU,const Eigen::MatrixXi& F,Eigen::MatrixXd &Delta_d_n){
    
    using namespace Eigen;
    
    //Make a linear system by measureing volumes and areas;
    VectorXd  volumes;
    
    VectorXd d_vector;
    MatrixXd area_matrix;
    volumes_and_areas(targetU,volumes,area_matrix);
    VectorXd volume_difference=initialVolumes-volumes;
    //Solve Linear system to find pressures of bubbles.
    d_vector = area_matrix.fullPivLu().solve(volume_difference);
    int nv=(int)targetU.rows();

    //Collect t/q-junctions.
    std::vector<bool> is_tq_junc(nv);
    for(int vi=0;vi<nv;++vi){
        is_tq_junc[vi]=mesh().is_vertex_nonmanifold(vi);
    }

    //Label of the incident regions for each vertex.
    std::vector<LosTopos::Vec2i> Vlabels(nv);
    for(int vi=0;vi<nv;++vi){
        if(is_tq_junc[vi]){
            continue;
        }
        
        assert(mesh().m_vertex_to_triangle_map[vi].size()>0);
        size_t tri=mesh().m_vertex_to_triangle_map[vi][0];
        Vlabels[vi]=mesh().get_triangle_label(tri);
        
    }

    //compute current normals.
    //Orientations of normals are from small-indexed region to large-indexed region.
    MatrixXd Normals;
    igl::per_vertex_normals(targetU, F, Normals);
    Delta_d_n=MatrixXd::Zero(nv, 3);
    
    //For non-tq-junctions,
    std::vector<int>non_t_junctions;
    
    for(int vi=0;vi<nv;++vi){
        
        if(is_tq_junc[vi]){
            continue;
        }
        
        const auto &v_labels=Vlabels[vi];

        int region0=(v_labels)[0];
        int region1=(v_labels)[1];

        int non_air_region;
        bool both_non_air=false;
        if(region0<=AIR){
            
            //Both sides of the vertex are open regions.
            if(region1<=AIR){
                continue;
            }
            
            non_air_region=region1;
            
        }else if(region1<=AIR){
            non_air_region=region0;
        }else{
            both_non_air=true;
        }
        
        if(!both_non_air){
            
            Delta_d_n.row(vi)=d_vector[non_air_region-region_offset]*Normals.row(vi);
        }
        
        if(both_non_air){
            int large_region_ind=region0>region1?region0:region1;
            int small_region_ind=region0<region1?region0:region1;

            Delta_d_n.row(vi)=(d_vector[large_region_ind-region_offset]-d_vector[small_region_ind-region_offset])*Normals.row(vi);
        }
        
    }

    //Special care for t/q-junctions.
    //Needs to be true when handling junctions accurately.
    if(delta_dn_tq_juctions){
        
        //For triple junctions
        for(size_t v=0;v<nv;++v){
            
            if(!is_tq_junc[v]){
                continue;
            }
            
            Delta_d_n.row(v)<<0,0,0;
            
            std::vector<size_t>adjacent_vertices;
            mesh().get_adjacent_vertices(v, adjacent_vertices);

            for(size_t ad_v:adjacent_vertices){
                if(!mesh().is_vertex_nonmanifold(ad_v)){
                    Delta_d_n.row(v)+=Delta_d_n.row(ad_v);
                }
            }

            Delta_d_n.row(v)/=adjacent_vertices.size();
            
        }
        
        //For quad or higher-junctions
        std::vector<size_t>q_junctions;
        
        for(size_t v=0;v<nv;++v){
            if(!is_tq_junc[v]){
                continue;
            }
            auto incident_edges=mesh().m_vertex_to_edge_map[v];
            int num_incident_edges=incident_edges.size();
            
            int t_junc_counter=0;
            for(size_t ei=0;ei<num_incident_edges;++ei){
                size_t edge=incident_edges[ei];
                if(mesh().is_edge_nonmanifold(edge)){
                    ++t_junc_counter;
                }
            }

            if(t_junc_counter>3 and !m_st->vertex_is_all_solid(v)){
                q_junctions.push_back(v);

            }

        }

        int num_qj=q_junctions.size();
        for(size_t qi=0;qi<num_qj;++qi){
            size_t q=q_junctions[qi];
            Delta_d_n.row(q)<<0,0,0;
            
            std::vector<size_t>adjacent_vertices;
            mesh().get_adjacent_vertices(q, adjacent_vertices);
            
            const bool consider_only_t_juncs=true;
            if(!consider_only_t_juncs){
                //Take the average of all the neighbor vertices.
                for(size_t ad_v:adjacent_vertices){
                    Delta_d_n.row(q)+=Delta_d_n.row(ad_v);
                    
                }
            }
            
            else {
                //Take the average of only the t-junc neighborfoods.
                auto incident_edges=mesh().m_vertex_to_edge_map[q];
                int num_incident_edges=incident_edges.size();
                
                for(size_t ei=0;ei<num_incident_edges;++ei){
                    size_t edge=incident_edges[ei];
                    if(mesh().is_edge_nonmanifold(edge)){
                        size_t ad_v=edge_other_vertex(ei, q);
                        Delta_d_n.row(q)+=Delta_d_n.row(ad_v);
                    }
                }
            }

            Delta_d_n.row(q)/=adjacent_vertices.size();
            
        }
        
    }

}


void SZHSolver::writeObj_FaceLabel_constrainedVertices(const bool with_imaginary_vertices){
    
    const bool with_normal=false;
    const bool change_y_z=true;
    MeshIO::saveOBJ(*this, "mesh.obj",with_imaginary_vertices,with_normal,change_y_z);

    std::string labelfile="flabel.txt";
    std::ofstream ofs_flabel(labelfile);
    
    int nt=(int)mesh().nt();
    ofs_flabel<<num_region<<std::endl;
    ofs_flabel<<nt<<std::endl;
    for(int i=0;i<nt;++i){
        
        LosTopos::Vec2i label = mesh().get_triangle_label(i);
        ofs_flabel<<label[0]<<" "<<label[1]<<std::endl;
    }

    //Write constrained vertices
    int nv_const=m_constrained_vertices.size();
    
    std::string constvertexfile="constvertex.txt";
    std::ofstream ofs_constvertex(constvertexfile);
    ofs_constvertex<<nv_const<<std::endl;
    for(int i=0;i<nv_const;++i){
        
        ofs_constvertex<<m_constrained_vertices[i]<<std::endl;
    }

}

void SZHSolver::write_constrained_mesh(std::string outputfile){
    
    if(Constrained_V.size()>0){
        
        Eigen::MatrixXd y_z_changed_Constrained_V(Constrained_V.rows(),3);
        y_z_changed_Constrained_V<<Constrained_V.col(0),Constrained_V.col(2),Constrained_V.col(1);
        
        igl::writeOBJ(outputfile,y_z_changed_Constrained_V,Constrained_F);
    }
}

void SZHSolver::write_film_mesh(std::string outputfile){
    
    MeshIO::saveOBJ(*this, outputfile,false,false,true);
}

void SZHSolver::write_mesh(){
    static int count=0;

    std::string output_directory="outputmesh/";
    std::stringstream output_meshfile;
    output_meshfile<<output_directory<<"output_mesh"<<std::setfill('0') << std::setw(6)<<count<<".obj";

    std::stringstream constrained_meshfile;
    constrained_meshfile<<output_directory<<"output_frame"<<std::setfill('0') << std::setw(6)<<count<<".obj";
    
    write_film_mesh(output_meshfile.str());
    write_constrained_mesh(constrained_meshfile.str());
    ++count;
}


void SZHSolver::total_area(double time,bool write){
    
    using namespace Eigen;
    double area=0;
    
    int nt=(int)mesh().nt();
    for (size_t t = 0; t < nt; t++)
    {
        
        if(m_st->triangle_is_all_solid(t)){
            continue;
        }
        auto f=mesh().get_triangle(t);

        LosTopos::Vec3d p0 = m_st->pm_positions[f.v[0]];
        LosTopos::Vec3d  p1 = m_st->pm_positions[f.v[1]];
        LosTopos::Vec3d  p2 = m_st->pm_positions[f.v[2]];
        
        LosTopos::Vec3d n=LosTopos::cross(p1-p0, p2-p0);

        double local_area=LosTopos::mag(n);
        area+=local_area;

    }
    
    area/=2.0;
    
    std::cout<<"total_area:"<<area<<std::endl;

    static std::string directory="./";
    static std::string filename="area_transition.csv";
    
    if(write){
        
        static std::ofstream output(directory+filename);
        static int count=0;
        
        output<<time<<","<<area<<std::endl;
        
        int max_count=-1;
        if(count++==max_count){
            output.close();
            exit(0);
            std::cout<<"exit in write_total_area";
        }
        
    }
    
}

void SZHSolver::easy_orientation(){
    //Set each triangle label (i,j) to be i>j.
    
    int nt=mesh().nt();
    for(int ti=0;ti<nt;++ti){
        int &label0=mesh().m_triangle_labels[ti][0];
        int &label1=mesh().m_triangle_labels[ti][1];
        
        if(label0<label1){
            std::swap( mesh().m_tris[ti][0],mesh().m_tris[ti][1]);
            std::swap( label0,label1);
        }
    }
}

void SZHSolver::volumes_and_areas(const Eigen::MatrixXd &targetU , Eigen::VectorXd &volumes, Eigen::MatrixXd& area_matrix){
        using namespace Eigen;
    
    volumes=VectorXd::Zero(num_bubbles);
    area_matrix=MatrixXd::Zero(num_bubbles,num_bubbles);

    int nt=(int)mesh().nt();
    for (size_t t = 0; t < nt; t++)
    {

        auto f=mesh().get_triangle(t);
        Eigen::Vector3d p0 = targetU.row(f.v[0]);
        Eigen::Vector3d p1 = targetU.row(f.v[1]);;
        Eigen::Vector3d p2 = targetU.row(f.v[2]);

        Eigen::Vector3d n = (p1-p0).cross(p2-p0);
        
        int label0=mesh().get_triangle_label(t)[0];
        int label1=mesh().get_triangle_label(t)[1];
        
        double local_area=n.norm();

        if(label0<=AIR and label1>AIR){
            area_matrix(label1-region_offset,label1-region_offset)+=local_area;
        }else if(label1<=AIR and label0>AIR){
            area_matrix(label0-region_offset,label0-region_offset)+=local_area;
        }else if(label0>AIR and label1>AIR){
            area_matrix(label0-region_offset,label1-region_offset)+=local_area;
            area_matrix(label1-region_offset,label0-region_offset)+=local_area;
        }

        // total volume via divergence theorem: ∫ 1
        double local_volume=n.dot(p0);
        if(label0>AIR){
            volumes[label0-region_offset]+=local_volume;
        }
        if(label1>AIR){
            volumes[label1-region_offset]-=local_volume;
            
        }
    }

    volumes/=6.0;
    area_matrix/=2.0;

    for(int i=0;i<num_bubbles;++i){
        area_matrix.row(i)*=-1;
        area_matrix(i,i)=std::abs(area_matrix.row(i).sum());
    }

}

void SZHSolver::volumes_and_areas( Eigen::VectorXd &volumes, Eigen::MatrixXd& area_matrix){

    using namespace Eigen;
    
    volumes=VectorXd::Zero(num_bubbles);
    area_matrix=MatrixXd::Zero(num_bubbles,num_bubbles);

    int nt=(int)mesh().nt();
    for (size_t t = 0; t < nt; t++)
    {

        auto f=mesh().get_triangle(t);

        LosTopos::Vec3d p0 = m_st->pm_positions[f.v[0]];
        LosTopos::Vec3d  p1 = m_st->pm_positions[f.v[1]];
        LosTopos::Vec3d  p2 = m_st->pm_positions[f.v[2]];
        
        LosTopos::Vec3d n=LosTopos::cross(p1-p0, p2-p0);

        int label0=mesh().get_triangle_label(t)[0];
        int label1=mesh().get_triangle_label(t)[1];

        double local_area=LosTopos::mag(n);

        if(label0<=AIR and label1>AIR){
            area_matrix(label1-region_offset,label1-region_offset)+=local_area;
        }else if(label1<=AIR and label0>AIR){
            area_matrix(label0-region_offset,label0-region_offset)+=local_area;
        }else if(label0>AIR and label1>AIR){
            area_matrix(label0-region_offset,label1-region_offset)+=local_area;
            area_matrix(label1-region_offset,label0-region_offset)+=local_area;
        }
        
        // total volume via divergence theorem: ∫ 1
        double local_volume=LosTopos::dot(n,p0);

        if(label0>AIR){
            volumes[label0-region_offset]+=local_volume;
        }
        if(label1>AIR){
            volumes[label1-region_offset]-=local_volume;
            
        }
        
    }

    volumes/=6.0;
    
    area_matrix/=2.0;

    for(int i=0;i<num_bubbles;++i){
        area_matrix.row(i)*=-1;
        area_matrix(i,i)=std::abs(area_matrix.row(i).sum());
    }

}

