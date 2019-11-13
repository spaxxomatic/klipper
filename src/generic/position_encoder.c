struct pos {
int32_t pos_x;
int32_t pos_y;
int32_t pos_z;
};
//Resolution/increment of the magnetic AS5311 sensor, in mm/step
#define AS5311_RESOLUTION 0.0004882
static float as5311resolution = AS5311_RESOLUTION;

typedef struct  {
      char name;
      int last_error;
      int error;
      uint16_t prev_enc_value;
      uint16_t enc_value;
      long abs_position;
      float position_mm;
} axis_struct;
      
static axis_struct X_axis = {.name = 'X', .enc_value = 0, .prev_enc_value = 0 , .error = 0, .abs_position = 0, .position_mm=0};

static axis_struct Y_axis = {.name = 'Y', .enc_value = 0, .prev_enc_value = 0 , .error = 0, .abs_position = 0, .position_mm=0};

void getEncoderData(char b){
    if 
}