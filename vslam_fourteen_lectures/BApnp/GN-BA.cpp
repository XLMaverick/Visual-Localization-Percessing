#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    ifstream p3dfile;
    p3dfile.open(p3d_file);
    if(!p3dfile)
    {
        cout<<"open p3d.txt error!"<<endl;
    }
    string p3dline;
    while(getline(p3dfile,p3dline) && !p3dline.empty())
    {
        istringstream p3dtempLine(p3dline);
        Vector3d p3dtemp;
        p3dtempLine>>p3dtemp[0]>>p3dtemp[1]>>p3dtemp[2];
        p3d.push_back(p3dtemp);
        
    }
    
    ifstream p2dfile;
    p2dfile.open(p2d_file);
    if(!p2dfile)
    {
        cout<<"open p2d.txt error!"<<endl;
    }
    string p2dline;
    while(getline(p2dfile,p2dline) && !p2dline.empty())
    {
        istringstream p2dtempLine(p2dline);
        Vector2d p2dtemp;
        p2dtempLine>>p2dtemp[0]>>p2dtemp[1];
        p2d.push_back(p2dtemp);
        
    }

    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti(Matrix3d::Identity(),Vector3d::Zero()); // estimated pose

    for (int iter = 0; iter < iterations; iter++) 
    {
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        Vector2d e;
        cost = 0;
        for (int i = 0; i < nPoints; i++) 
        {
            Vector3d pc = T_esti*p3d[i];
            Vector3d e_temp = Vector3d(p2d[i][0],p2d[i][1],1) - K*pc/pc[2];
            e[0] = e_temp[0];
            e[1] = e_temp[1];
            cost  += 0.5*e.transpose()*e; 
            double x = pc[0], y = pc[1], z = pc[2];
            Matrix<double, 2, 6> J;
            J(0,0) = -fx/z;
            J(0,2) = fx*x/(z*z);
            J(0,3) = fx*x*y/(z*z);
            J(0,4) = -fx - fx*(x*x)/(z*z);
            J(0,5) =  fx*y/z;
            J(1,1) = -fy/z;
            J(1,2) = fy*y/(z*z);
            J(1,3) = fy+fy*(y*y)/(z*z);
            J(1,4) = -fy*x*y/(z*z);
            J(1,5) = -fy*x/z;

            H +=J.transpose()*J;
            b +=-J.transpose()*e;
        }
        Vector6d dx;
        dx = H.ldlt().solve(b);

        if (isnan(dx[0])) 
        {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) 
        {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        T_esti = Sophus::SE3::exp(dx)*T_esti;
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
