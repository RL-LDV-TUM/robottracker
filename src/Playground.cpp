#include "Playground.hpp"

Playground::Playground(float width, float height, float cellLength) : aruco::Marker(), psize(cv::Size(width, height)), cellLength(cellLength)
{
  id = 0;
}

/*
* Draw playground
*/
void Playground::draw(cv::Mat &in, cv::Scalar color, int lineWidth) const
{
    if (size()!=4) return;
    cv::line( in,(*this)[0],(*this)[1],color,lineWidth,CV_AA);
    cv::line( in,(*this)[1],(*this)[2],color,lineWidth,CV_AA);
    cv::line( in,(*this)[2],(*this)[3],color,lineWidth,CV_AA);
    cv::line( in,(*this)[3],(*this)[0],color,lineWidth,CV_AA);
    
    putText(in, "A", (*this)[0], cv::FONT_HERSHEY_SIMPLEX, 2, color, 1, CV_AA);
    putText(in, "B", (*this)[1], cv::FONT_HERSHEY_SIMPLEX, 2, color, 1, CV_AA);
    putText(in, "C", (*this)[2], cv::FONT_HERSHEY_SIMPLEX, 2, color, 1, CV_AA);
    putText(in, "D", (*this)[3], cv::FONT_HERSHEY_SIMPLEX, 2, color, 1, CV_AA);
    
    // draw 3 axes
    /*
    std::vector<cv::Point3f> axes = {cv::Point3f(0,0,0),
                                     cv::Point3f(10,0,0), cv::Point3f(0,10,0), cv::Point3f(0,0,10)};
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axes, Rvec, Tvec, cameraParameters.CameraMatrix, cameraParameters.Distorsion, imagePoints);
    
    cv::line( in, imagePoints[0],imagePoints[1],cv::Scalar(0,0,255),lineWidth,CV_AA);
    cv::line( in, imagePoints[0],imagePoints[2],cv::Scalar(255,0,0),lineWidth,CV_AA);
    cv::line( in, imagePoints[0],imagePoints[3],cv::Scalar(0,255,0),lineWidth,CV_AA);
    */

}

/*
* Calc playground extrinsics
*/
void Playground::calculateExtrinsics(const aruco::CameraParameters &CP)throw(cv::Exception)
{
    if (!isValid()) throw cv::Exception(9004,"!isValid(): invalid marker. It is not possible to calculate extrinsics","calculateExtrinsics",__FILE__,__LINE__);
    if (psize.width <=0 || psize.height <= 0) throw cv::Exception(9004,"width <=0 || height <= 0: invalid markerSize","calculateExtrinsics",__FILE__,__LINE__);
    if (!CP.isValid()) throw cv::Exception(9004,"!CP.isValid(): invalid camera parameters. It is not possible to calculate extrinsics","calculateExtrinsics",__FILE__,__LINE__);
    if (CP.CameraMatrix.rows==0 || CP.CameraMatrix.cols==0) throw cv::Exception(9004,"CameraMatrix is empty","calculateExtrinsics",__FILE__,__LINE__);
    
    cv::Size halfSize = cv::Size(psize.width*0.5f, psize.height*0.5f);

    cv::Mat ObjPoints(4,3,CV_32FC1);
    ObjPoints.at<float>(0,0)=-halfSize.width;
    ObjPoints.at<float>(0,1)=-halfSize.height;
    ObjPoints.at<float>(0,2)=0;
    ObjPoints.at<float>(1,0)=-halfSize.width;
    ObjPoints.at<float>(1,1)=halfSize.height;
    ObjPoints.at<float>(1,2)=0;
    ObjPoints.at<float>(2,0)=halfSize.width;
    ObjPoints.at<float>(2,1)=halfSize.height;
    ObjPoints.at<float>(2,2)=0;
    ObjPoints.at<float>(3,0)=halfSize.width;
    ObjPoints.at<float>(3,1)=-halfSize.height;
    ObjPoints.at<float>(3,2)=0;


    cv::Mat ImagePoints(4,2,CV_32FC1);

    //Set image points from the marker
    for (int c=0;c<4;c++)
    {
        ImagePoints.at<float>(c,0)=(*this)[c%4].x;
        ImagePoints.at<float>(c,1)=(*this)[c%4].y;
    }
    
    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImagePoints, CP.CameraMatrix, CP.Distorsion, raux, taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec,CV_32F);
    
}

/*
* Draw Playground locked label
* TODO: own class for labels
*/
void Playground::drawLockedLabel(cv::Mat &image, cv::Point pos)
{
  putText(image, "Playground Locked [l]", pos, 
           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
}
