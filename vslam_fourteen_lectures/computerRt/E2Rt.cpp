#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/so3.h>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    JacobiSVD<MatrixXd> svd(E,ComputeThinU | ComputeThinV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Matrix3d Sigma = U.inverse()*E*V.transpose().inverse();
    // cout<<"U:\n"<<U<<"\nV:\n"<<V<<"\nSigma:\n"<<Sigma<<endl;
    vector<double>  tao = {Sigma(0,0),Sigma(1,1),Sigma(2,2)};
    sort(tao.begin(), tao.end());
    Matrix3d SigmaFix = Matrix3d::Zero();
    double tao_mead = (tao[1]+tao[2])*0.5;
    SigmaFix(0,0) = tao_mead;
    SigmaFix(1,1) = tao_mead;
    cout<<"Sigma after fix: \n"<<SigmaFix<<endl;

    Matrix3d R_Z1 = AngleAxisd(M_PI/2, Vector3d(0,0,1)).matrix();
    Matrix3d R_Z2 = AngleAxisd(-M_PI/2, Vector3d(0,0,1)).matrix();

    Matrix3d t_wedge1 = U*R_Z1*SigmaFix*U.transpose();
    Matrix3d t_wedge2 = U*R_Z2*SigmaFix*U.transpose();

    Matrix3d R1 = U*R_Z1*V.transpose();
    Matrix3d R2 = U*R_Z2*V.transpose(); 


    cout << "R1 = " << endl<<R1 << endl;
    cout << "R2 = " << endl<<R2 << endl;
    cout << "t1 = " <<endl<< Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " <<endl<< Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}