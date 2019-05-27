#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>



using namespace std;
using namespace Eigen;


string trajectory_file = "../compare.txt";




double timestamp_g ,timestamp_e;
Eigen::Vector3d tg, te;
Eigen::Quaterniond qg, qe;
Sophus::SE3 Tg , Te;


// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2);

void pose_estimation_3d3d (const vector<Eigen::Vector3d>& pts1,
    const vector<Eigen::Vector3d>& pts2,
    Eigen::Matrix3d& R, Eigen::Vector3d& t
);

void bundleAdjustment (
    const vector< Eigen::Vector3d>& pts1,
    const vector< Eigen::Vector3d >& pts2,
    Eigen::Matrix3d& R, Eigen::Vector3d& t );


int main(int argc, char **argv) 
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g, poses_e, poses_c;

    vector<Eigen::Vector3d> position_g, position_e;
    ifstream trajectory;
    trajectory.open(trajectory_file.c_str());

    if(!trajectory)
    {
        cout<<"open file error!"<<endl;
        return -1;
    }
    string trajectoryLine;
    while(getline(trajectory, trajectoryLine) && !trajectoryLine.empty())
    {
        istringstream templine(trajectoryLine);
        templine>>timestamp_e>>te[0]>>te[1]>>te[2]>>qe.x()>>qe.y()>>qe.z()>>qe.w()
                >>timestamp_g>>tg[0]>>tg[1]>>tg[2]>>qg.x()>>qg.y()>>qg.z()>>qg.w();
        Te = Sophus::SE3(qe,te);
        Tg = Sophus::SE3(qg,tg);
        position_g.push_back(tg);
        position_e.push_back(te);
        poses_g.push_back(Tg);
        poses_e.push_back(Te);
    }
    trajectory.close();
    // DrawTrajectory(poses_g, poses_e);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    // pose_estimation_3d3d(position_g, position_e, R,t);
    bundleAdjustment(position_g, position_e, R,t);

    Sophus::SE3 Tge(R,t);
    
    // cout<<"the T is "<<endl<<Tge.matrix()<<endl;
    for(int i= 0;i< poses_e.size();i++)
    {
        poses_c.push_back(Tge*poses_e[i]);
    }
    DrawTrajectory(poses_g, poses_c);

    return 0;
}


class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};


void bundleAdjustment (
    const vector< Eigen::Vector3d>& pts1,
    const vector< Eigen::Vector3d >& pts2,
    Eigen::Matrix3d& R, Eigen::Vector3d& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(pts2[i]);
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( pts1[i]);
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true ); //可以打印出优化过程信息
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    // cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

    R = Eigen::Isometry3d( pose->estimate() ).rotation();
    t[0] = Eigen::Isometry3d( pose->estimate() ).matrix()(0,3);
    t[1] = Eigen::Isometry3d( pose->estimate() ).matrix()(1,3);
    t[2] = Eigen::Isometry3d( pose->estimate() ).matrix()(2,3);

}

void pose_estimation_3d3d (const vector<Eigen::Vector3d>& pts1,
    const vector<Eigen::Vector3d>& pts2,
    Eigen::Matrix3d& R, Eigen::Vector3d& t
)
{
    Eigen::Vector3d p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Eigen::Vector3d( p1 /  N);
    p2 = Eigen::Vector3d( p2 / N);
    vector<Eigen::Vector3d>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += q1[i]*q2[i].transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}
    
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    R = U* ( V.transpose() );
    t = p1- R * p2;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2) 
{
    if (poses1.empty()) {
        cerr << "Trajectory 1 is empty!" << endl;
        return;
    }

    if (poses2.empty()) {
        cerr << "Trajectory 2 is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses1.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            // glColor3f(1 - (float) i / poses1.size(), 0.0f, (float) i / poses1.size());
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < poses2.size() - 1; i++) {
             glColor3f(0.0f, 0.0f,1.0f);
            // glColor3f(1 - (float) i / poses2.size(), 0.0f, (float) i / poses2.size());
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}