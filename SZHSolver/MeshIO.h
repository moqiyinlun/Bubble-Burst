#ifndef __MeshIO__
#define __MeshIO__

#include <iostream>
#include <vector>
#include "surftrack.h"
#include "SZHSolver.h"

class MeshIO
{
public:
    static bool load(SZHSolver & hgf, const std::string & filename, bool binary = true);
    static bool saveOBJ(SZHSolver & hgf, const std::string & filename,const bool with_imaginary_vertices=true, const bool with_normal=true, const bool change_y_z=false);

};

#endif /* defined(__MeshIO__) */
