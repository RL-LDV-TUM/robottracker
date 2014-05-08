#include "Playground.hpp"

Playground::Playground()
{

}

       
/**Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
 * @param markerSize size of the marker side expressed in meters
 * @param CP parmeters of the camera
 * @param setYPerperdicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
 */
void Playground::calculateExtrinsics(cv::Size markerSize,const aruco::CameraParameters &CP,bool setYPerperdicular) throw(cv::Exception)
{

}

void Playground::draw(cv::Mat &in, cv::Scalar color, int lineWidth) const
{
    if (size()!=4) return;
    cv::line( in,(*this)[0],(*this)[1],color,lineWidth,CV_AA);
    cv::line( in,(*this)[1],(*this)[2],color,lineWidth,CV_AA);
    cv::line( in,(*this)[2],(*this)[3],color,lineWidth,CV_AA);
    cv::line( in,(*this)[3],(*this)[0],color,lineWidth,CV_AA);

}
