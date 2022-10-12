#include "BundleAdjuster.h"


/*#include <map>
#include <vector>

#include <opencv2/core/core.hpp>

#include <pcl/PCLHeader.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace pcl;

void BundleAdjuster::adjustBundle(std::vector<CloudPoint>& pointcloud, const cv::Mat& cam_intrinsics,
                                  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
                                  std::map<int ,cv::Matx34d>& Pmats)
{
    int N = Pmats.size(), M = pointcloud.size(), K = -1;
    std::cout<<"N (cams) = "<< N <<" M (points) = "<< M <<" K (measurements) =
    "<< K <<endl;
    StdDistortionFunction distortion;
    // intrinsic parameters matrix
    Matrix3x3d KMat;
    makeIdentityMatrix(KMat);
    KMat[0][0] = cam_intrinsics.at<double>(0,0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, orig_cloud;

    KMat[0][1] = cam_intrinsics.at<double>(0,1);
    KMat[0][2] = cam_intrinsics.at<double>(0,2);
    KMat[1][1] = cam_intrinsics.at<double>(1,1);
    KMat[1][2] = cam_intrinsics.at<double>(1,2);
    //.....

    // 3D point cloud
    std::vector<Vector3d >Xs(M);
    for (int j = 0; j < M; ++j)
    {
        Xs[j][0] = pointcloud[j].pt.x;
        Xs[j][1] = pointcloud[j].pt.y;
        Xs[j][2] = pointcloud[j].pt.z;
    }
    cout<<"Read the 3D points."<<endl;
    // convert cameras to BA datastructs
    vector<CameraMatrix> cams(N);
    for (inti = 0; i< N; ++i)
    {
        intcamId = i;
        Matrix3x3d R;
        Vector3d T;
        Matx34d& P = Pmats[i];
        R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); T[0] = P(0,3);
        R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); T[1] = P(1,3);
        R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); T[2] = P(2,3);
        cams[i].setIntrinsic(Knorm);
        cams[i].setRotation(R);
        cams[i].setTranslation(T);
    }
    cout<<"Read the cameras."<<endl;

    vector<Vector2d > measurements;
    vector<int> correspondingView;
    vector<int> correspondingPoint;
    // 2D corresponding points
    for (unsigned int k = 0; k <pointcloud.size(); ++k)
    {
        for (unsigned int i=0; i<pointcloud[k].imgpt_for_img.size(); i++)
        {
            if (pointcloud[k].imgpt_for_img[i] >= 0)
            {
                int view = i, point = k;
                Vector3d p, np;
                Point cvp = imgpts[i][pointcloud[k].imgpt_for_img[i]].pt;
                p[0] = cvp.x;
                p[1] = cvp.y;
                p[2] = 1.0;
                // Normalize the measurements to match the unit focal length.
                scaleVectorIP(1.0/f0, p);
                measurements.push_back(Vector2d(p[0], p[1]));
                correspondingView.push_back(view);
                correspondingPoint.push_back(point);
            }
        }
     } // end for (k)
     K = measurements.size();
     cout<<"Read "<< K <<" valid 2D measurements."<<endl;

     // perform the bundle adjustment
     {
         CommonInternalsMetricBundleOptimizeropt(V3D::FULL_BUNDLE_FOCAL_
         LENGTH_PP, inlierThreshold, K0, distortion, cams, Xs, measurements,
         correspondingView, correspondingPoint);
         opt.tau = 1e-3;
         opt.maxIterations = 50;
         opt.minimize();
         cout<<"optimizer status = "<<opt.status<<endl;
     }
     //...
     //extract 3D points
     for (unsigned int j = 0; j <Xs.size(); ++j)
     {
        pointcloud[j].pt.x = Xs[j][0];
        pointcloud[j].pt.y = Xs[j][1];
        pointcloud[j].pt.z = Xs[j][2];
     }
    //extract adjusted cameras
    for (int i = 0; i< N; ++i)
    {
        Matrix3x3d R = cams[i].getRotation();
        Vector3d T = cams[i].getTranslation();
        Matx34d P;
        P(0,0) = R[0][0]; P(0,1) = R[0][1]; P(0,2) = R[0][2]; P(0,3) = T[0];
        P(1,0) = R[1][0]; P(1,1) = R[1][1]; P(1,2) = R[1][2]; P(1,3) = T[1];
        P(2,0) = R[2][0]; P(2,1) = R[2][1]; P(2,2) = R[2][2]; P(2,3) = T[2];
        Pmats[i] = P;
    }
}




*/
