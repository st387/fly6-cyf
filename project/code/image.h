/*
 * image.h
 *
 *  Created on: 2026年1月9日
 *      Author: 30382
 */

#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_


#define MAX_LABELS  250
extern uint8 image[MT9V03X_H][MT9V03X_W];
extern uint8 image_zip[MT9V03X_H/2][MT9V03X_W/2];
extern uint8 image_bin[MT9V03X_H][MT9V03X_W];
extern uint8 image_sobel[MT9V03X_H][MT9V03X_W];
extern uint8 image_out[MT9V03X_H][MT9V03X_W];
extern uint8_t labels[MT9V03X_H][MT9V03X_W];


int OtsuAlgThreshold(uint8 *image);
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);

void zip_image(uint8_t *image1);
void bin_image(uint8 T);



typedef struct {
    int sum_x;      // 连通域内所有像素的x坐标总和
    int sum_y;      // 连通域内所有像素的y坐标总和
    int pixel_count;// 连通域像素数量（用于计算质心）
    int max_x;
    int max_y;
    int min_x;
    int min_y;
} Blob;
typedef struct {

float x;

float y;

} Point;


extern uint8_t root_map[MAX_LABELS];
extern uint8_t blob_num;
extern uint8_t parent[MAX_LABELS];
extern uint8_t labels[MT9V03X_H][MT9V03X_W];
typedef struct
{
uint8 x;
uint8 y;
} point;

// 定义连通域类型
typedef enum {
    BLOB_NOISE = 0,      // 噪点（太小）
    BLOB_SINGLE_CIRCLE,  // 单个圆（信标）
    BLOB_MULTI_CIRCLE,   // 三个小圆的组合
    BLOB_UNKNOWN         // 无法确定
} BlobType;

extern uint8_t beacon_only[MT9V03X_H][MT9V03X_W];
extern Point max_point;
void fuck_zaodian(const point lu, const point rd,
                  const uint8 img[MT9V03X_H][MT9V03X_W],
                  uint8 out[MT9V03X_H][MT9V03X_W]);
void sobel(uint8_t *src, uint8_t *edges, int width, int height, int threshold);
uint8_t two_pass(uint8_t image[MT9V03X_H][MT9V03X_W], Point *min_point, float* out_xx,float* out_yy);
extern uint8_t dothang[200];
extern uint8_t dotlie[200];
extern uint8_t aroundTime; // 记录未检测到白点的时间计数器
extern uint8_t aroundFlag;
extern uint8_t amendFlag; // 标志位，用于判断是否需要调整转向策略
typedef struct {
    int Ture_Error;      // 转向偏差值
    int Ture_Error_V;    // 可能是速度相关的误差调整值
    int speed;           // 车辆当前速度
} Car_Price_Type;
void Seek_Beacon ();
void find_real_beacons(float *out_x,float *out_y) ;
int is_part_of_three_circles(int blob_idx) ;
int find_car();

#endif /* CODE_IMAGE_H_ */
