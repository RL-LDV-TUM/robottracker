#ifndef _Playground_H_
#define _Playground_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>

typedef std::vector<std::vector<cv::Point> > Contours;


class Playground : public aruco::Marker {


  public:
  
    Playground(float width = -1, float height = -1);
    ~Playground() {}

    void calculateExtrinsics(float markerSize, const aruco::CameraParameters &CP, bool setYPerperdicular=true) throw(cv::Exception) {};
    
    void calculateExtrinsics(const aruco::CameraParameters &CP) throw(cv::Exception);

    void draw(cv::Mat &in, cv::Scalar color, int lineWidth) const;
    
    
  protected:
  
    cv::Size psize;

};

#endif
