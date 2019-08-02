#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file_g = "../groundtruth.txt";
double timestamp_g;
Eigen::Vector3d tg;
Eigen::Quaterniond qg;
Sophus::SE3 Tg;


string trajectory_file_e = "../estimated.txt";
double timestamp_e;
Eigen::Vector3d te;
Eigen::Quaterniond qe;
Sophus::SE3 Te;

Eigen::Matrix<double,6,1> kesi;
double error = 0;
double RMSE;

int main(int argc, char **argv) 
{
    // vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    ifstream trajectory_g;
    ifstream trajectory_e;
    trajectory_g.open(trajectory_file_g.c_str());
    trajectory_e.open(trajectory_file_e.c_str());

    if(!trajectory_g || ! trajectory_e)
    {
        cout<<"open file error!"<<endl;
        return -1;
    }
    int num = 0;
    string trajectoryLine_g;
    string trajectoryLine_e;
    while(getline(trajectory_g, trajectoryLine_g) && !trajectoryLine_g.empty() &&
            getline(trajectory_e,trajectoryLine_e) && !trajectoryLine_e.empty())
    {
        istringstream templine_g(trajectoryLine_g);
        istringstream templine_e(trajectoryLine_e);
        templine_g>>timestamp_g>>tg[0]>>tg[1]>>tg[2]>>qg.x()>>qg.y()>>qg.z()>>qg.w();
        templine_e>>timestamp_e>>te[0]>>te[1]>>te[2]>>qe.x()>>qe.y()>>qe.z()>>qe.w();
        Tg = Sophus::SE3(qg,tg);
        Te = Sophus::SE3(qe,te);

        kesi = (Tg.inverse()*Te).log();
        error += kesi.transpose()*kesi;
        num++; 
    }
    trajectory_g.close();
    trajectory_e.close();
    RMSE = sqrt(error/num);
    cout<<"RMSE: "<<RMSE<<endl;
    return 0;
}