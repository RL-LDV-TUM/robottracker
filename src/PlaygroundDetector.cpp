#include "PlaygroundDetector.hpp"
#include <limits>
#include <opencv2/highgui/highgui.hpp>

PlaygroundDetector::PlaygroundDetector()
{

}

/*
* Find Playground in image and calc its Extrinsics
*/
bool PlaygroundDetector::detect(const cv::Mat &image, Playground &playground, Contours &candidateContours, aruco::CameraParameters &cameraParameters)
{
  // pg not valid
  playground.id = -1;

  // get all contours from color thres image
  Contours contours;
  findContours(image, contours);
  
  // search for L shaped contours
  filterContours(contours, candidateContours);
  
    if(candidateContours.size() < 4) return false;
  
  // combine exatly 4 L-contours to one rectangle with 4 corners
  std::vector<cv::Point2f> corners; // max length: 4
  extractPlayGroundCorners(candidateContours, corners);
  
    if(corners.size() != 4) return false;
  
  playground.resize(4); 
  std::copy(corners.begin(), corners.end(), playground.begin());
  
  // playground valid
  playground.id = 0;
  
  // calc translation and rotation
  playground.calculateExtrinsics(cameraParameters);
  
  return true;
}

/*
* Find all contours in image
*/
void PlaygroundDetector::findContours(const cv::Mat &image, Contours &contours) const
{
  cv::Mat hsv, thres;
  cv::cvtColor ( image, hsv, CV_BGR2HSV );
  
  // light blue: hue 80-120
  cv::inRange(hsv, cv::Scalar(80, 40, 40), cv::Scalar(120, 255, 255), thres);
  
  //erode(thres, thres, cv::Mat());
  
  cv::findContours(thres, contours, cv::noArray(), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  
}

/*
* Filter for L-shaped contours
*/
void PlaygroundDetector::filterContours(const Contours &contours, Contours &filteredContours) const
{

  filteredContours.clear();

  for(unsigned i=0; i<contours.size(); i++)
  {
    if(contours[i].size() > 5)
    {
      double area = cv::contourArea(contours[i]);
      if(area > 100 && area < 3000)
      {
        // smooth contour
        std::vector<cv::Point> approxCurve;
        cv::approxPolyDP(contours[i], approxCurve, double ( contours[i].size() ) *0.025 , true );
        
        if(approxCurve.size() != 6) continue;
        
        std::vector<cv::Point> hull;
        cv::convexHull(approxCurve, hull);
        
        float convexity = area / cv::contourArea(hull);
        
        if(convexity > 0.3f && convexity < 0.7f && hull.size() == 5)
        {
          filteredContours.push_back(approxCurve);
          //std::cout << "Area:" << area << " Convexity:" << convexity << std::endl;
        }
      }
    }
  }
  
}

/*
* Combine exatly 4 L-contours to one rectangle with 4 corners
*/
void PlaygroundDetector::extractPlayGroundCorners(const Contours &filteredContours, std::vector<cv::Point2f> &corners) const
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
    

        
