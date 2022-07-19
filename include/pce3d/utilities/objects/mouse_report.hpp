#ifndef mouse_report_hpp
#define mouse_report_hpp


namespace pce {

struct MouseReport{
  int x_position;
  int y_position;
  int prev_x_pos;
  int prev_y_pos;
  bool left_clicked;
  bool right_clicked;
};


}


#endif /* mouse_report_hpp */
