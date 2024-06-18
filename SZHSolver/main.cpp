#include <iostream>
#include <sstream>
#include "Options.h"
#include "Sim.h"
#include "MeshIO.h"
#include <sys/stat.h>
#include <sys/types.h>
Sim g_sim(false);


bool createDirectory(const std::string& path) {
    if (mkdir(path.c_str(), 0755) == -1) {
        if (errno == EEXIST) {
            return true;
        } else {
            return false;
        }
    }
    return true;
}


void parse_arguments(int argc, char **argv,std::string*inputdata_dir,int* duration,std::string* output_dir){
    
    using std::cout;
    using std::endl;
    using std::string;
    using std::stringstream;
    
    for(int i=1;i<argc;++i){
        
        stringstream stream(argv[i]);
        string attribute_name;
        string attribute_value;
        
        getline(stream,attribute_name,'=');
        getline(stream,attribute_value,'=');

        if(attribute_name=="inputdata_dir"){
            *inputdata_dir= attribute_value;
        }else if(attribute_name=="duration"){
            *duration = atoi(attribute_value.c_str());
        }else if(attribute_name=="output_path"){
            *output_dir = attribute_value;
        }

    }

}

int main(int argc, char * argv[])
{
   
    std::string inputdata_dir="";
    int duration_time = 0;
    std::string output_dir = "";
    parse_arguments(argc, argv,&inputdata_dir,&duration_time,&output_dir);
    if (createDirectory(output_dir)) {
        std::cout << "Directory created: " << output_dir << std::endl;
    } else {
        std::cerr << "Failed to create directory: " << output_dir << std::endl;
    }
    bool success = g_sim.init(argv[1],inputdata_dir);
    std::cout << "Initialization complete. Starting the simulation..." << std::endl;
    std::stringstream ss;
    ss <<output_dir<< "/mesh_" << g_sim.step_number << ".obj";
    std::string meshname = ss.str();
    ss.str("");
    ss << output_dir<< "/label_" << g_sim.step_number << ".txt";
    std::string labelname = ss.str();
    ss.str("");
    g_sim.get_hgf()->saveResult(false, meshname, labelname);
    // while (true){
    for(int i = 0;i <duration_time; i++){
        g_sim.step();
        std::stringstream ss;
        ss <<output_dir<< "/mesh_" << g_sim.step_number << ".obj";
        std::string meshname = ss.str();
        ss.str("");
        ss << output_dir<< "/label_" << g_sim.step_number << ".txt";
        std::string labelname = ss.str();
        ss.str("");
        g_sim.get_hgf()->saveResult(false, meshname, labelname);
        if (g_sim.isFinished()){
            g_sim.get_hgf()->saveResult(false, meshname, labelname);
            break;
        }
    }
    
}

