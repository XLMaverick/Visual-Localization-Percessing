#include<iostream>
#include<ctime>
#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;

#define SIZE 100

int main(int argc, char** argv)
{
    /* Eigen最大的固定数组为50，当超过50时，如果没有使用动态初始化的方式，可以运行，但是时间效率非常的慢*/
    // Eigen::Matrix<double, SIZE, SIZE> matrixA;
    // Eigen::Matrix<double, SIZE,1> matrixb;

    //两种动态动态初始化的方式。
    // Eigen::MatrixXd matrixA;
    // Eigen::MatrixXd matrixb;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrixA;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrixb;

    matrixA = Eigen::MatrixXd::Random(SIZE,SIZE);
    matrixb = Eigen::MatrixXd::Random(SIZE,1);

    /*将系数矩阵变换为正定对称矩阵，如果没有这一步，Cholesky分解与前两个结果不一致 */
    matrixA = matrixA.transpose()*matrixA;

    Eigen::Matrix<double,SIZE,1> x;
    clock_t time_sst = clock();
    x = matrixA.inverse()*matrixb;
    cout<<x.transpose()<<endl;
    cout<<"time use in normal inverse is "<<1000*(clock()-time_sst)/(double)CLOCKS_PER_SEC<<"ms"<<endl;


    time_sst = clock();
    x= matrixA.colPivHouseholderQr().solve(matrixb);
    cout<<x.transpose()<<endl;
    cout<<"time use in QR decomposition is "<< 1000*(clock()-time_sst)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

    time_sst = clock();
    x = matrixA.ldlt().solve(matrixb);
    cout<<x.transpose()<<endl;
    cout<<"time use in Cholesky decomposition is "<<1000*(clock()-time_sst)/(double)CLOCKS_PER_SEC<<"ms"<<endl;

    return 0;
}