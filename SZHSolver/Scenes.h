#ifndef __MultiTracker__Scenes__
#define __MultiTracker__Scenes__

#include <iostream>
#include "SZHSolver.h"

class SZHScenes
{
public:
    // scene-specific initialization
    static SZHSolver * sceneInputData        (Sim * sim, std::vector<LosTopos::Vec3d> & vs, std::vector<LosTopos::Vec3st> & fs, std::vector<LosTopos::Vec2i> & ls, std::vector<size_t> & cv, std::vector<Vec3d> & cx,const std::string inputdata_dir="");
    static void burstBubbles               (double dt, Sim * sim, SZHSolver * hgf);
    static void pullBubbles             (double dt, Sim * sim, SZHSolver * hgf);
    static void volume_change(double dt, Sim * sim, SZHSolver * hgf);
    static double frame_center_x;

};

#endif /* defined(__MultiTracker__Scenes__) */
