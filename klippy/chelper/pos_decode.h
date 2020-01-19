
typedef struct {
    int xerror;
    int yerror;
    float xposition;
    float yposition;
} axis_position_struct;
axis_position_struct get_axis_stat();
float get_x_pos();
float get_y_pos();
void receive_position_info(int id, uint8_t *buf, int length);
void zero_axis(char axis);