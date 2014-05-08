#ifndef _Playground_H_
#define _Playground_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>

typedef std::vector<std::vector<cv::Point> > Contours;


class Playground : public std::vector<cv::Point2f> {


  public:
  
    Playground();
    ~Playground() {}

    /**Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
     * @param markerSize size of the marker side expressed in meters
     * @param CP parmeters of the camera
     * @param setYPerperdicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
     */
    void calculateExtrinsics(cv::Size markerSize, const aruco::CameraParameters &CP, bool setYPerperdicular=true)throw(cv::Exception);

    void draw(cv::Mat &in, cv::Scalar color, int lineWidth) const;
    
    
  private:
    

};

#endif
