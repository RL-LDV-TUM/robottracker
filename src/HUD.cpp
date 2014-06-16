#include "HUD.hpp"

HUD::HUD(cv::Mat &image, cv::Point legendPos) : image(image), legendPos(legendPos)
{

}

/*
* Draw playground, color according to lock / newly detected
*/
void HUD::drawPlayground(const Playground &playground, bool highlight, bool pglock)
{
  cv::Scalar color = (pglock) ? cv::Scalar(0,255,0) : cv::Scalar(0,134,209);
  int lineSize = (highlight) ? 4 : 1;
  playground.draw(image,color,lineSize);
}

/*
* Draw all detected markers
*/
void HUD::drawMarkers(const std::vector<aruco::Marker> &markers)
{
  for (unsigned i=0;i<markers.size();i++)
  {
      markers[i].draw(image,cv::Scalar(0,0,255),1);
  }
}

/*
* Show shortcuts and toggled flags
*/
void HUD::drawLegend(bool pause, bool pglock)
{
  cv::Point pos = legendPos;
  int font = cv::FONT_HERSHEY_SIMPLEX;
  cv::Scalar color(0,255,0);
  int thickness = 2;
  int linetype = CV_AA;
  
  std::string text;
  
  text = (pglock) ? "Playground *LOCKED*, unlock: [l]" : "Playground lock: [l]";
  putText(image, text, pos, font, 1, color, thickness, linetype);
  pos.y += 30;
  
  text = (pause) ? "*PAUSED*, continue: [p]" : "Pause: [p]";
  putText(image, text, pos, font, 1, color, thickness, linetype);
  pos.y += 30;
  
}

