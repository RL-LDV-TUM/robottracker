#ifndef _Playground_H_
#define _Playground_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>

typedef std::vector<std::vector<cv::Point> > Contours;


class Playground : public aruco::Marker {


  public:
  
    Playground(float width = -1, float height = -1, float cellLength = 10);
    ~Playground() {};

    /*
    * Override marker extrinsics calc
    */
    void calculateExtrinsics(float markerSize, const aruco::CameraParameters &CP, bool setYPerperdicular=true) throw(cv::Exception) {};
    
    /*
    * Calc playground extrinsics
    */
    void calculateExtrinsics(const aruco::CameraParameters &CP) throw(cv::Exception);

    /*
    * Draw playground
    */
    void draw(cv::Mat &in, cv::Scalar color, int lineWidth, const aruco::CameraParameters &CP) const;
    
    /*
    * Draw Playground locked label
    * TODO: own class for labels
    */
    static void drawLockedLabel(cv::Mat &image, cv::Point pos);
    
    float getWidth() const {return psize.width;}
    float getHeight() const {return psize.height;}
    float getCellLength() const {return cellLength;}
    
  protected:
  
    cv::Size psize;
    float cellLength;

};

#endif
