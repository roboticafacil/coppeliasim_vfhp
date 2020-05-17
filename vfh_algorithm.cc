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

#include "vfh/vfh_algorithm.h"

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <iostream>

VFH_Algorithm::VFH_Algorithm(VFHParams_t &vfhParams, MotionParams_t &motionParams): _vfhParams(vfhParams), _motionParams(motionParams)
{

}

/**
 * Class destructor
 */
VFH_Algorithm::~VFH_Algorithm()
{

}

void VFH_Algorithm::Init()
{
    int x,y;
    int cells2;
    double dx,dy,cell_enlarge;
    double dmax,dmax2;
    std::vector<unsigned int> temp_sector(2);
    cells=(unsigned int)(floor(_vfhParams.windows_size/_vfhParams.cell_size));
    hist_size=(unsigned int)(floor(2.0*M_PI/_vfhParams.sector_angle));
    dmax=double(cells)*sqrt(2.0)/2.0;
    dmax2=dmax*dmax;
#ifdef VFHp_DEBUG
    std::cout << "cells: " << cells << std::endl;
    std::cout << "hist_size: " << hist_size << std::endl;
    std::cout << "dmax: " << dmax << std::endl;
#endif
    cell_dist.clear();
    cell_ang.clear();
    cell_mag.clear();
    cell_sectors.clear();
    cell_hist.clear();
    primary_hist.clear();
    binary_hist.clear();
    masked_hist.clear();
    candidates.clear();
    temp_sector[0]=0;
    temp_sector[1]=0;
    std::vector<double> row(cells);
    for (unsigned int i=0;i<cells;i++)
        row[i]=0.0;
    std::vector<std::vector<unsigned int> > temp_sectors(cells);
    for (unsigned int i=0;i<cells;i++)
        temp_sectors[i]=temp_sector;
    for (unsigned int i=0;i<cells;i++)
    {
        cell_dist.push_back(row);
        cell_ang.push_back(row);
        cell_mag.push_back(row);
        cell_hist.push_back(row);
        cell_sectors.push_back(temp_sectors);
    }
    for (unsigned int i=0;i<hist_size;i++)
    {
        primary_hist.push_back(0);
        binary_hist.push_back(false);
        masked_hist.push_back(false);
    }
    cells2=cells/2;
    for (int i=0;i<cells;i++)
    {
        for (int j=0;j<cells;j++)
        {
            x=i-cells2;
            y=j-cells2;
            dx=double(x);
            dy=double(y);
            cell_dist[i][j]=sqrt(dx*dx+dy*dy);
            cell_ang[i][j]=atan2(dy,dx);
            cell_mag[i][j]=100.0*((dmax-cell_dist[i][j])*(dmax-cell_dist[i][j]))/dmax2;
            if (cell_dist[i][j]>0.0)
                cell_enlarge=asin(MAX(MIN(_vfhParams.safety_dist/cell_dist[i][j]/_vfhParams.cell_size,1.0),-1.0));
            else
                cell_enlarge=0.0;
            cell_sectors[i][j][0]=Angle2Sector(cell_ang[i][j]-cell_enlarge);
            cell_sectors[i][j][1]=Angle2Sector(cell_ang[i][j]+cell_enlarge);
            cell_hist[i][j]=0.0;
        }
    }
}

void VFH_Algorithm::ComputeDirection(std::vector<bool> &masked, Pose_t &robot, Point_t &goal)
{
    target_candidate=WrapAngle(atan2(goal.y-robot.y,goal.x-robot.x)-robot.th);
    FindCandidates(masked);
    SelectCandidate();
    previous_candidate=selected_candidate;
}

void VFH_Algorithm::ComputeHistogram(std::vector<Point_t > &laser, std::vector<Arc_t > &sonar)
{
    CellHist(laser,sonar);
    PrimaryHist();
    BinaryHist();
    MaskedHist();
}

NHMotion_t VFH_Algorithm::ComputeMotion(double velocity, double direction, Pose_t &robot, Point_t &goal)
{
    NHMotion_t motion;
    motion.wL=0.0;
    motion.wR=0.0;
    double d=sqrt((robot.x-goal.x)*(robot.x-goal.x)+(robot.y-goal.y)*(robot.y-goal.y));
    if (d>_motionParams.stop_dist)
    {
        motion.wL=(velocity/(_motionParams.off_center_dist*_motionParams.wheel_radius))*(_motionParams.off_center_dist*cos(direction)-_motionParams.wheel_base*sin(direction));
        motion.wR=(velocity/(_motionParams.off_center_dist*_motionParams.wheel_radius))*(_motionParams.off_center_dist*cos(direction)+_motionParams.wheel_base*sin(direction));
    }
    return motion;
}

double VFH_Algorithm::getSelectedDirection()
{
    return selected_candidate;
}

double VFH_Algorithm::getTargetDirection()
{
    return target_candidate;
}

unsigned int VFH_Algorithm::Angle2Sector(double angle)
{
    int sector;
    double s=(angle/_vfhParams.sector_angle);
    if (s<0.0)
        sector=hist_size-1+ceil(s);
    else
        sector=floor(s);
    return (unsigned int)(sector);
}

void VFH_Algorithm::CellHist(std::vector<Point_t > &laser, std::vector<Arc_t > &sonar)
{
    double r;
    int x,y;
    double cells2;
    r=_vfhParams.safety_dist/_vfhParams.cell_size;
    cells2=double(cells/2);
    for (unsigned int i=0;i<cells;i++)
    {
        for (unsigned int j=0;j<cells;j++)
            cell_hist[i][j]=0;
    }
    for (unsigned int i=0;i<laser.size();i++)
    {
        x=(unsigned int)(floor(laser[i].x/_vfhParams.cell_size+cells2));
        y=(unsigned int)(floor(laser[i].y/_vfhParams.cell_size+cells2));
        if ((x>=0)&&(y>=0)&&(x<cells)&&(y<cells))
        {
            if (cell_dist[x][y]>r)
                cell_hist[x][y]=cell_hist[x][y]+cell_mag[x][y];
            else
                cell_hist[x][y]=cell_hist[x][y]+_vfhParams.tau_high;
        }
    }
    for (unsigned int i=0;i<sonar.size();i++)
    {
        for (int angle=-sonar[i].beam_angle/2;angle<=sonar[i].beam_angle/2;angle+=_vfhParams.sector_angle/2)
        {
            x=(unsigned int)(floor((sonar[i].r*cos(sonar[i].th+angle))/_vfhParams.cell_size+cells2));
            y=(unsigned int)(floor((sonar[i].r*sin(sonar[i].th+angle))/_vfhParams.cell_size+cells2));
            if ((x>=0)&&(y>=0)&&(x<cells)&&(y<cells))
            {
                if (cell_dist[x][y]>r)
                    cell_hist[x][y]=cell_hist[x][y]+cell_mag[x][y];
                else
                    cell_hist[x][y]=cell_hist[x][y]+_vfhParams.tau_high;
            }
        }
    }
}


void VFH_Algorithm::PrimaryHist()
{
    for (unsigned int i=0;i<hist_size;i++)
        primary_hist[i]=0;
    for (unsigned int i=0;i<cells;i++)
    {
        for (unsigned int j=0;j<cells;j++)
        {
            if (cell_hist[i][j]>0.0)
            {
                if (cell_sectors[i][j][0]>cell_sectors[i][j][1])
                {
                    for (unsigned int k=0;k<=cell_sectors[i][j][1];k++)
                        primary_hist[k]+=cell_hist[i][j];
                    for (unsigned int k=cell_sectors[i][j][0];k<hist_size;k++)
                        primary_hist[k]+=cell_hist[i][j];
                }
                else
                {
                    for (unsigned int k=cell_sectors[i][j][0];k<=cell_sectors[i][j][1];k++)
                        primary_hist[k]+=cell_hist[i][j];
                }
            }
        }
    }
}

void VFH_Algorithm::BinaryHist()
{
    for (unsigned int i=0;i<hist_size;i++)
    {
        if (primary_hist[i]>=_vfhParams.tau_high)
            binary_hist[i]=true;
        else if (primary_hist[i]<_vfhParams.tau_high)
            binary_hist[i]=false;
    }
}

void VFH_Algorithm::MaskedHist()
{
    unsigned int blind_ini,blind_end;
    for (unsigned int k=0;k<hist_size;k++)
        masked_hist[k]=binary_hist[k];
    blind_ini=Angle2Sector(_vfhParams.scan_angle/2.0);
    blind_end=Angle2Sector(M_2PI-_vfhParams.scan_angle/2.0);
    for (unsigned int k=blind_ini;k<blind_end;k++)
        masked_hist[k]=true;
}

void VFH_Algorithm::FindCandidates(std::vector<bool> &masked)
{
    int hist_size;
    candidates.clear();
    int last_sector, target_sector;
    std::vector<int> valley_ini;
    std::vector<int> valley_end;
    valley_ini.clear();
    valley_end.clear();
    hist_size=masked.size();
    if (hist_size==int(this->hist_size))
    {
        last_sector=hist_size-1;
        if (!masked[0])
        {
            for (int i=(hist_size-1);i>=0;i--)
            {
                if (masked[i])
                {
                    valley_ini.push_back((i-hist_size)%hist_size);
                    last_sector=i;
                    break;
                }
            }
        }
        for (unsigned int i=1;i<=last_sector;i++)
        {
            if ((masked[i-1])&&(!masked[i]))
                valley_ini.push_back(i);
            else if ((!masked[i-1])&&(masked[i]))
                valley_end.push_back(i-1);
        }
        if ((!masked[hist_size-1])&&(masked[0]))
                valley_end.push_back(hist_size-1);
        for (unsigned int i=0;i<valley_ini.size();i++)
        {
            if (fabs(valley_end[i]-valley_ini[i])>=_vfhParams.wide_valley_width)
            {
                candidates.push_back((double(valley_ini[i])+0.5)*_vfhParams.sector_angle);
                candidates.push_back((double(valley_end[i])+0.5)*_vfhParams.sector_angle);
            }
            else
                candidates.push_back((double(valley_ini[i]+valley_end[i])/2.0+0.5)*_vfhParams.sector_angle);
        }
        target_sector=int(Angle2Sector(target_candidate));
        if (!masked[target_sector])
            candidates.push_back(target_candidate);
        for (unsigned int i=0;i<candidates.size();i++)
            candidates[i]=WrapAngle(candidates[i]);
    }
}

double VFH_Algorithm::WrapAngle(double angle)
{
    if (angle>M_PI)
        angle=angle-M_2PI;
    else if (angle<-M_PI)
        angle=angle+M_2PI;
    return angle;
}

void VFH_Algorithm::SelectCandidate()
{
    unsigned int best;
    double best_value,value;
    best=0;
    selected_candidate=0;
    if (candidates.size()>0)
    {
        best_value=EvaluateCandidate(candidates[0]);
        for (unsigned int i=1;i<candidates.size();i++)
        {
            value=EvaluateCandidate(candidates[i]);
            if (value<best_value)
            {
                best=i;
                best_value=value;
            }
        }
        selected_candidate=candidates[best];
    }
}

double VFH_Algorithm::EvaluateCandidate(double candidate)
{
    double value1,value2,value3;
    value1=_vfhParams.target_weight*fabs(WrapAngle(candidate-target_candidate));
    value2=_vfhParams.current_weight*fabs(candidate);
    value3=_vfhParams.previous_weight*fabs(WrapAngle(candidate-previous_candidate));
    return value1+value2+value3;
}

