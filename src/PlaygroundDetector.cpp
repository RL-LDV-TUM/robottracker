#include "PlaygroundDetector.hpp"
#include <limits>

PlaygroundDetector::PlaygroundDetector()
{

}

bool PlaygroundDetector::detect(const cv::Mat &thresImage, Playground &playground, cv::Mat &img)
{
  Contours contours;
  findContours(thresImage, contours);
  
  Contours filteredContours;
  filterContours2(contours, filteredContours);
  
  // TODO: remove!
  cv::drawContours(img, contours, -1, cv::Scalar(0,0,255), 1);
  
  if(filteredContours.size() != 4) return false;
  
  std::vector<cv::Point2f> corners; // max length: 4
  extractCorners(filteredContours, corners);
  
  // TODO: Rotation?
  
  std::cout << corners[0] << corners[1] << corners[2] << corners[3] << playground.size() << std::endl;
  playground.resize(4);
  std::copy(corners.begin(), corners.end(), playground.begin());
  std::cout << playground[0] << playground[1] << playground[2] << playground[3] << std::endl;

  return true;
}


void PlaygroundDetector::findContours(const cv::Mat &thresImage, Contours &contours) const
{
  cv::findContours(thresImage, contours, cv::noArray(), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
}

void PlaygroundDetector::filterContours(const Contours &contours, Contours &filteredContours) const
{

  filteredContours.clear();

  for(unsigned i=0; i<contours.size(); i++)
  {
    if(contours[i].size() > 20)
    {
      double area = cv::contourArea(contours[i]);
      if(area > 300 && area < 1300)
      {
        std::vector<cv::Point> approxCurve;
        cv::approxPolyDP(contours[i], approxCurve, double ( contours[i].size() ) *0.01 , true );
        
        if(approxCurve.size() != 6) continue;
        
        cv::Rect bounding = cv::boundingRect(approxCurve);
        float convexity = area / bounding.area();
        float ratio = (float) bounding.height / bounding.width;
        
        if(convexity > 0.05 && convexity < 0.5 && ratio > 0.85 && ratio < 1.15)
        {
          filteredContours.push_back(approxCurve);
          std::cout << "Area:" << area << " Convexity:" << convexity << " Ratio:" << bounding.height << "-" << bounding.width << "=" << ratio << std::endl;
        }
      }
    }
  }
  
}

void PlaygroundDetector::extractCorners(const Contours &filteredContours, std::vector<cv::Point2f> &corners) const
{

    corners.clear();
    
    // sum up points to get center
    for(std::vector<cv::Point> contour : filteredContours)
    {
      cv::Point sum = std::accumulate(contour.begin(), contour.end(), cv::Point(0, 0));
      cv::Point center(sum.x / contour.size(), sum.y / contour.size());
       
      cv::Point &nearest = contour[0];
      float minDist = std::numeric_limits<float>::max();
      
      // select point nearest to center
      for(cv::Point point : contour)
      {
        float dist = cv::norm(point - center);
        if(dist < minDist)
        {
          minDist = dist;
          nearest = point;
        }
      }
      corners.push_back(cv::Point2f(nearest.x, nearest.y));
    }
}
    

        
