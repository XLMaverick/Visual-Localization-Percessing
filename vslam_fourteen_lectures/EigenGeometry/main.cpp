#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{

    /*四元数定义的顺序和打印的顺序不一致，需要注意*/
    Eigen::Quaterniond q1(0.7,0.3,0.2,0.2);
    cout<<"quaterniond q1 "<<q1.coeffs();
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    /*单位四元数才能描述旋转，所以使用四元数前必须进行归一化*/
    T1.rotate(q1.normalized().toRotationMatrix());
    T1.pretranslate(Eigen::Vector3d(0.7,1.1,0.2));

    Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);
    cout<<"quaterniond q2 "<<q2.coeffs();
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T2.rotate(q2.normalized().toRotationMatrix());
    T2.pretranslate(Eigen::Vector3d(-0.1,0.4,0.8));

    Eigen::Vector3d p1(0.5,-0.1,0.2);
    Eigen::Vector3d p2 = Eigen::Vector3d::Identity();

    p2 = T2*T1.inverse()*p1;

    cout<<"position in camera2 is "<<p2.transpose()<<endl;
    return 0;
}