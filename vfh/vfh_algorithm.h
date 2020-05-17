/*
 *  Orca-Components: Components for robotics.
 *
 *  Copyright (C) 2004
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef VFH_ALGORITHM_H
#define VFH_ALGORITHM_H

#include <vector>
#include <stdio.h>
#include <iostream>

#define VFHp_DEBUG

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifndef M_2PI
    #define M_2PI (2.0*M_PI)
#endif

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#define DTOR(d) ((d) * M_PI / 180)
#define RTOD(r) ((r*180.0)/M_PI)

typedef struct VFHParams{
    double scan_angle;
    double cell_size;
    double windows_size;
    double sector_angle;
    double safety_dist;
    unsigned int wide_valley_width;
    double tau_low;
    double tau_high;
    double target_weight;
    double previous_weight;
    double current_weight;
}VFHParams_t;

typedef struct MotionParams{
    double stop_dist;
    double off_center_dist;
    double wheel_radius;
    double wheel_base;
}MotionParams_t;

typedef struct Pose{
    double x;
    double y;
    double th;
}Pose_t;

typedef struct Point_t{
    double x;
    double y;
}Point_t;

typedef struct Arc{
    double r;
    double th;
    double beam_angle;
}Arc_t;

typedef struct NHMotion{
    double wL;
    double wR;
}NHMotion_t;


class VFH_Algorithm
{
public:
    VFH_Algorithm(VFHParams_t &vfhParams, MotionParams_t &motionParams);
    ~VFH_Algorithm();
    void Init();
    void ComputeDirection(std::vector<bool> &masked, Pose_t &robot, Point_t &goal);
    void ComputeHistogram(std::vector<Point_t > &laser, std::vector<Arc_t > &sonar);
    NHMotion_t ComputeMotion(double velocity, double direction, Pose_t &robot, Point_t &goal);
    double getSelectedDirection();
    double getTargetDirection();
    std::vector<std::vector<double> > cell_hist;
    std::vector<double> primary_hist;
    std::vector<bool> binary_hist;
    std::vector<bool> masked_hist;
    std::vector<double> candidates;

private:
    VFHParams_t _vfhParams;
    MotionParams_t _motionParams;
    Pose_t robot;
    Point_t goal;
    std::vector<std::vector<double> > cell_dist;
    std::vector<std::vector<double> > cell_ang;
    std::vector<std::vector<double> > cell_mag;
    std::vector<std::vector<std::vector<unsigned int> > > cell_sectors;

    double target_candidate;
    double previous_candidate;
    double selected_candidate;
    unsigned int cells;
    unsigned int hist_size;
    void CellHist(std::vector<Point_t > &laser, std::vector<Arc_t > &sonar);
    void PrimaryHist();
    void BinaryHist();
    void MaskedHist();
    void FindCandidates(std::vector<bool> &masked);
    void SelectCandidate();
    double EvaluateCandidate(double candidate);
    unsigned int Angle2Sector(double angle);
    double WrapAngle(double angle);
};

#endif
