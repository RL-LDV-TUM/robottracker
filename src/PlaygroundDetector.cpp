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
  filterContours(contours, filteredContours);
  
    // TODO: remove!
    cv::drawContours(img, filteredContours, -1, cv::Scalar(0,0,255), 1);
  
  if(filteredContours.size() < 4) return false;
  
  std::vector<cv::Point2f> corners; // max length: 4
  extractCorners(filteredContours, corners);
  
  if(corners.size() != 4) return false;
  
  playground.resize(4); 
  std::copy(corners.begin(), corners.end(), playground.begin());

  return true;
}


void PlaygroundDetector::findContours(const cv::Mat &thresImage, Contours &contours) const
{
  cv::Mat thres = thresImage.clone();
  cv::findContours(thres, contours, cv::noArray(), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
}

void PlaygroundDetector::filterContours(const Contours &contours, Contours &filteredContours) const
{

  filteredContours.clear();

  for(unsigned i=0; i<contours.size(); i++)
  {
    if(contours[i].size() > 20)
    {
      double area = cv::contourArea(contours[i]);
      if(area > 200 && area < 1500)
      {
        std::vector<cv::Point> approxCurve;
        cv::approxPolyDP(contours[i], approxCurve, double ( contours[i].size() ) *0.01 , true );
        
        if(approxCurve.size() != 6) continue;
        
        std::vector<cv::Point> hull;
        cv::convexHull(approxCurve, hull);
        
        float convexity = area / cv::contourArea(hull);
        
        if(convexity > 0.25f && convexity < 0.45f && hull.size() == 5)
        {
          filteredContours.push_back(approxCurve);
          //std::cout << "Area:" << area << " Convexity:" << convexity << std::endl;
        }
      }
    }
  }
  
}

void PlaygroundDetector::filterContours2(const Contours &contours, Contours &filteredContours) const
{

  filteredContours.clear();

  for(unsigned i=0; i<contours.size(); i++)
  {
    if(contours[i].size() > 20)
    {
      double area = cv::contourArea(contours[i]);
      if(area > 300 && area < 1500)
      {
        std::vector<cv::Point> approxCurve;
        cv::approxPolyDP(contours[i], approxCurve, double ( contours[i].size() ) *0.01 , true );
        
        if(approxCurve.size() < 4 || approxCurve.size() > 6) continue;
        
        std::vector<cv::Point> hull;
        cv::convexHull(approxCurve, hull);
        
        cv::Rect bounding = cv::boundingRect(approxCurve);
        float convexity = area / cv::contourArea(hull);
        float ratio = (float) bounding.height / bounding.width;
        
        if(convexity > 0.15f && convexity < 0.45f && ratio > 0.8f && ratio < 1.2f)
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
    
    cv::convexHull(corners, corners, true);
    cv::approxPolyDP(corners, corners, 50, true );
    
    if(corners.size() != 4) return;
    
    // find left upper corner = A
    std::vector<cv::Point2f>::iterator A = corners.begin();
    float minCoords = std::numeric_limits<float>::max();
    
    for(auto point = corners.begin(); point != corners.end(); point++)
    {
       float coords = point->x + point->y;
       if(coords < minCoords)
       {
         minCoords = coords;
         A = point;
       }
    }
    
    // rotate, so that A is first
    // A - D
    // |   |
    // B - C
    std::rotate(corners.begin(), A, corners.end());
    
    // rotation: restrict to 180 rotation
    // determine rotation by playground side lengths
    double a = cv::norm(corners[0] - corners[1]);
    double b = cv::norm(corners[1] - corners[2]);
    
    if(a > b) // rotate 90° if AB is longer than BC
      std::rotate(corners.begin(), corners.begin() + 1, corners.end() );
    
}
    

        
