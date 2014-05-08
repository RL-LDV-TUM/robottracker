#ifndef _PlaygroundDetector_H_
#define _PlaygroundDetector_H_

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <vector>

#include "Playground.hpp"

typedef std::vector<std::vector<cv::Point> > Contours;


class PlaygroundDetector {


  public:
     
    PlaygroundDetector();
    ~PlaygroundDetector() {}

    bool detect(const cv::Mat &thresImage, Playground &Playground, cv::Mat &img);
        

  private:
    void findContours(const cv::Mat &thresImage, Contours &contours) const;
    void filterContours(const Contours &contours, Contours &filteredContours) const;
    void extractCorners(const Contours &filteredContours, std::vector<cv::Point2f> &corners) const;
    

};

#endif
