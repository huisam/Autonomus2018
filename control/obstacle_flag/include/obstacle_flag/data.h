#include <deque>
#include <vector>

struct active{
    bool active_dynamic;
    bool active_park;
    bool active_uturn;
};
struct priv{
    std::deque<double> priv_data_first_y;
    std::vector<double> delta_y;
};
struct dynamic{
    int size_N;
    int sample_size;
    double min_y_distance;
    double max_y_distance;
    double min_x_obstacle;
    double max_x_width;
    double min_delta_y;
    double max_delta_y;
};
struct u_turn_param{
    int check_UTurn;
    int number_of_uturn;
    int before_detect = 0;
    double UTurn_distance_point;
};
struct park{
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double obj_2_distance;
};