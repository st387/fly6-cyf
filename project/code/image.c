/*
 * image.c
 *
 *  Created on: 2026年1月9日
 *      Author: 30382
 */
/*
 * image.c
 *
 *  Created on: 2025年3月17日
 *      Author: GUO
 */
#include "zf_common_headfile.h"
uint8 image[MT9V03X_H][MT9V03X_W];
uint8 image_zip[MT9V03X_H/2][MT9V03X_W/2];
uint8 image_bin[MT9V03X_H][MT9V03X_W];
uint8 image_sobel[MT9V03X_H][MT9V03X_W];
uint8 image_out[MT9V03X_H][MT9V03X_W];
#define M_PI 3.1415926535
//***************Otsu算法通过求类间方差极大值求自适应阈值******************

/***************************************************************
* 函数名称：otsuThreshold(uint8 *image, uint16 col, uint16 row); 大津法
* 函数输入：摄像头传感器图像数组（mt9v03x_image）
* 函数输出：阈值大小 （Threshold）
* 功能说明：求阈值大小
***************************************************************/
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};          //存放每个像素值的个数
    uint32 Amount = 0;                      //像素点总数
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
    uint8 Threshold = 0;

    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y*Image_Width + X]]++;//统计每个灰度值的个数信息
        }    }

    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
    if(Threshold>88)
    {
        return Threshold;
    }
    else
    {
        return 88;
    }

}


//图像压缩

void zip_image(uint8_t *image) {
    for (int y = 0; y < MT9V03X_H / 2; ++y) {
        for (int x = 0; x < MT9V03X_W / 2; ++x) {
            uint8 *image1=image;
            // 计算 2x2 块的平均值
            uint8_t avg = (
                image1[(y * 2) * MT9V03X_W + (x * 2)] +
                image1[(y * 2) * MT9V03X_W + (x * 2) + 1] +
                image1[(y * 2 + 1) * MT9V03X_W + (x * 2)] +
                image1[(y * 2 + 1) * MT9V03X_W + (x * 2) + 1]
            ) / 4;

            // 存储到压缩后的图像数组
            image_zip[y][x] = avg;
        }
    }
}



////***************二值化******************
uint16 cut_image=15;//裁剪图像
void bin_image(uint8 T)
{
    for(uint16 k=65;k<MT9V03X_H/2+65;k++)
    {
        tft180_draw_point(k, (cut_image-1)/2, RGB565_RED);
    }
    for(int i=cut_image-1;i<MT9V03X_H ;i++)
     {
         for(int j=0;j<MT9V03X_W;j++)
         {
             if(image_sobel[i][j]>T)
             {
                 image_bin[i][j]=255;
             }
             else image_bin[i][j]=0;
         }
      }
}

//
//void bin_image(uint8 T)
//{
//    for(int i=0;i<MT9V03X_H/2 ;i++)
//     {
//         for(int j=0;j<MT9V03X_W/2;j++)
//         {
//             if(image_zip[i][j]>T)
//             {
//                 image_bin[i][j]=255;
//             }
//             else image_bin[i][j]=0;
//         }
//      }
//}
//
//

////
//// 去噪点，再写四向sobel
//// 去噪点
////

void fuck_zaodian(const point lu, const point rd,
                  const uint8 img[MT9V03X_H][MT9V03X_W],
                  uint8 out[MT9V03X_H][MT9V03X_W])
{
    for(uint8 i=((uint8)lu.y);i<=((uint8)rd.y);++i)
        for(uint8 j=((uint8)lu.x);j<=((uint8)rd.x);++j)
        {
            uint8 cnt=0;
            if(img[i][j]>=100)
            {
                cnt+=abs(img[i][j]-img[i-1][j-1])>10;
                cnt+=abs(img[i][j]-img[i-1][j+1])>10;
                cnt+=abs(img[i][j]-img[i+1][j-1])>10;
                cnt+=abs(img[i][j]-img[i+1][j+1])>10;
            }           out[i][j]=(cnt>=4?((img[i-1][j-1]+img[i-1][j]+img[i-1][j+1]+img[i][j-1]+img[i][j]+img[i][j+1]+img[i+1][j-1]+img[i+1][j]+img[i+1][j+1])/9):img[i][j]);
//遍历指定区域内的每一个像素，检查当前像素值是否大于等于100。如果是，则检查其四个对角线方向像素的差值是否超过10，统计这样的差异次数`cnt`。如果`cnt`大于等于4，就将当前像素替换为周围3x3区域的均值，否则保留原值。
        }
}

//四向sobel·

// Sobel卷积函数
/**
 * @brief 四向Sobel边缘检测（整合版）
 * @param src     输入图像（灰度图，8位无符号）
 * @param edges   输出边缘图（8位无符号，255为边缘，0为非边缘）
 * @param width   图像宽度
 * @param height  图像高度
 * @param threshold 边缘判定阈值（建议范围50~150）
 *
 * 功能说明：
 * 1. 对每个像素点，计算0°、45°、90°、135°四个方向的梯度。
 * 2. 取四个方向梯度的绝对值最大值，若超过阈值则判定为边缘。
 * 3. 使用定点数运算优化
 *
 * 优化点：
 * - 手动展开3x3卷积循环，减少循环次数。
 * - 使用Q7.8定点数格式（256表示1.0，-512表示-2.0），提升计算速度。
 * - 边界处理：跳过图像边缘（x,y从1开始到width-2/height-2）。
 */
void sobel(uint8_t *src, uint8_t *edges, int width, int height, int threshold) {
    // 定义四个方向的Sobel核（Q7.8定点数，256代表1.0，512代表2.0）
    const int16_t kernels[4][3][3] = {
        // 0°方向核（检测垂直边缘）
        { {-256, 0, 256}, {-512, 0, 512}, {-256, 0, 256} },
        // 45°方向核（检测135°边缘）
        { {-512, -256, 0}, {-256, 0, 256}, {0, 256, 512} },
        // 90°方向核（检测水平边缘）
        { {-256, -512, -256}, {0, 0, 0}, {256, 512, 256} },
        // 135°方向核（检测45°边缘）
        { {0, 256, 512}, {-256, 0, 256}, {-512, -256, 0} }
    };

    // 遍历图像中的每个像素（跳过边缘1像素）
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int max_gradient = 0; // 记录四个方向中的最大梯度值

            // 对四个方向分别计算梯度
            for (int dir = 0; dir < 4; dir++) {
                // 手动展开3x3卷积计算（避免嵌套循环）
                int32_t sum = 0;
                sum += src[(y-1)*width + (x-1)] * kernels[dir][0][0]; // 左上
                sum += src[(y-1)*width +  x   ] * kernels[dir][0][1]; // 中上
                sum += src[(y-1)*width + (x+1)] * kernels[dir][0][2]; // 右上
                sum += src[ y   *width + (x-1)] * kernels[dir][1][0]; // 左中
                sum += src[ y   *width +  x   ] * kernels[dir][1][1]; // 中心
                sum += src[ y   *width + (x+1)] * kernels[dir][1][2]; // 右中
                sum += src[(y+1)*width + (x-1)] * kernels[dir][2][0]; // 左下
                sum += src[(y+1)*width +  x   ] * kernels[dir][2][1]; // 中下
                sum += src[(y+1)*width + (x+1)] * kernels[dir][2][2]; // 右下

                // 取绝对值并右移8位（Q7.8转回实际值）
                int16_t gradient = abs((int16_t)(sum >> 8));

                // 更新最大梯度值
                if (gradient > max_gradient) {
                    max_gradient = gradient;
                }
            }

            // 应用阈值：超过阈值则为边缘（255），否则为背景（0）
            edges[y*width + x] = (max_gradient > threshold) ? 255 : 0;
        }
    }

    // 边缘填充：将图像四周边缘设为0
    for (int x = 0; x < width; x++) edges[x] = 0;                      // 上边缘
    for (int x = 0; x < width; x++) edges[(height-1)*width + x] = 0;   // 下边缘
    for (int y = 0; y < height; y++) edges[y*width] = 0;               // 左边缘
    for (int y = 0; y < height; y++) edges[y*width + (width-1)] = 0;   // 右边缘
}


//// 深度优先搜索函数，用于标记连通域并计算质心
//void dfs(int image[ROWS][COLS], int x, int y, int label, int* sumX, int* sumY, int* count) {
//    if (x < 0 || x >= ROWS || y < 0 || y >= COLS || image[x][y] != 1) return; // 越界或不是前景像素
//
//    // 标记当前点
//    image[x][y] = label;
//    *sumX += x;
//    *sumY += y;
//    (*count)++;
//
//    // 搜索相邻点
//    dfs(image, x+1, y, label, sumX, sumY, count);
//    dfs(image, x-1, y, label, sumX, sumY, count);
//    dfs(image, x, y+1, label, sumX, sumY, count);
//    dfs(image, x, y-1, label, sumX, sumY, count);
//}
//
//// 寻找图像中所有圆的中心点
//void find(int image[ROWS][COLS], Point centers[], int* centerCount) {
//    int label = 2; // 从2开始以避免与背景(0)和前景(1)混淆
//    *centerCount = 0;
//
//    for (int i = 0; i < ROWS; ++i) {
//        for (int j = 0; j < COLS; ++j) {
//            if (image[i][j] == 1) { // 找到未被标记的前景像素
//                int sumX = 0, sumY = 0, count = 0;
//                dfs(image, i, j, label++, &sumX, &sumY, &count);
//
//                if (count > 0) {
//                    centers[*centerCount].x = sumX / count;
//                    centers[*centerCount].y = sumY / count;
//                    (*centerCount)++;
//                }
//            }
//        }
//    }
//}
////比较圆心值
//int compare(const void *a, const void *b) {
//    Point pointA = *(Point *)a;
//    Point pointB = *(Point *)b;
//
//    // 首先按y轴（垂直方向）降序排列，即从下到上
//    if (pointA.y != pointB.y) return pointB.y - pointA.y;
//    // 如果y轴相同，则按x轴（水平方向）升序排列，即从左到右
//    return pointA.x - pointB.x;
//}
Blob blobs[MAX_LABELS];
Point max_point;  // 存储最大连通域的中心坐标
uint8_t parent[MAX_LABELS];
uint8_t labels[MT9V03X_H][MT9V03X_W];
uint8_t blob_num;
static uint8_t blob_count = 0;
uint8_t root_map[MAX_LABELS] = {0};
int turn_flag = 0,cheak_flag = 1,check_flag_cnt=0,turn_check=0;
uint8_t blob_Xmax,blob_Ymax;
uint8_t two_pass(uint8_t image[MT9V03X_H][MT9V03X_W], Point *min_point, float* out_xx,float* out_yy)
{

    // 清空临时数据
    memset(labels, 0, sizeof(labels));
    memset(parent, 0, sizeof(parent));
//    memset(blobs, 0, sizeof(blobs));

    uint8_t label = 1;
    blob_count = 0;

    // 第一次扫描：分配标签
    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            if (image[y][x] == 0) continue;

            uint8_t left = (x > 0) ? labels[y][x-1] : 0;
            uint8_t top = (y > 0) ? labels[y-1][x] : 0;

            if (left == 0 && top == 0) {
                if (label >= MAX_LABELS) return 0;
                labels[y][x] = label;
                parent[label] = label;
                label++;
            } else if (left != 0 && top == 0) {
                labels[y][x] = left;
            } else if (left == 0 && top != 0) {
                labels[y][x] = top;
            } else {
                uint8_t min_label = (left < top) ? left : top;
                uint8_t max_label = (left > top) ? left : top;
                labels[y][x] = min_label;
                if (max_label < MAX_LABELS) {
                    parent[max_label] = min_label;
                }
            }
        }
    }

    // 路径压缩优化
    for (uint8_t i = 1; i < label && i < MAX_LABELS; i++) {
        uint8_t safety_counter = 0;
        while (parent[i] < MAX_LABELS &&
               parent[parent[i]] < MAX_LABELS &&
               parent[i] != parent[parent[i]] &&
               safety_counter++ < MAX_LABELS) {
            parent[i] = parent[parent[i]];
        }
    }

    // 第二次扫描：统计连通域
    memset(root_map, 0, sizeof(root_map));
    uint8_t valid_blobs = 0;
    memset(blobs, 0, MAX_LABELS * sizeof(Blob));

    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            if (labels[y][x] == 0) continue;

            uint8_t root = parent[labels[y][x]];
            if (root == 0 || root >= MAX_LABELS || labels[y][x] >= MAX_LABELS) continue;

            if (root_map[root] == 0) {
                if (valid_blobs >= MAX_LABELS) {
                    blob_count = valid_blobs;
                    return 0;
                }
                valid_blobs++ ;
                root_map[root] = valid_blobs;
            }

            uint8_t blob_idx = root_map[root] - 1;  //转换0基
            
            if (x < blobs[blob_idx].min_x) blobs[blob_idx].min_x = x;
            if (x > blobs[blob_idx].max_x) blobs[blob_idx].max_x = x;
            if (y < blobs[blob_idx].min_y) blobs[blob_idx].min_y = y;
            if (y > blobs[blob_idx].max_y) blobs[blob_idx].max_y = y; 
            
            blobs[blob_idx].sum_x += x;
            blobs[blob_idx].sum_y += y;
            blobs[blob_idx].pixel_count++;
        }
    }

      blob_count = valid_blobs;
      blob_num = blob_count;
//  
//    float current_min = 99999.0f;
//      min_point->x = -1.0f;
//      min_point->y = -1.0f;
//
//      // 分块计算坐标
//      for (uint8_t i = 0; i < blob_count; i++) {
//          Blob *current_blob = &blobs[i];
//          if (current_blob->pixel_count == 0) continue; // 跳过无效Blob
//
//          float center_x = (float)current_blob->sum_x / current_blob->pixel_count;
//          float center_y = (float)current_blob->sum_y / current_blob->pixel_count;
//          float center=(center_x-100)*(center_x-100)+(90-center_y)*(90 -center_y);
//
//          if (current_min-center>30) {
//              current_min = center;
//              min_point->x = center_x;
//              min_point->y = center_y;
//          }
//      }
//    if (out_xx&&blob_count!=0) *out_xx = min_point->x;
//    *out_yy=min_point->y;
//    if(min_point->y>=90)
//    {
//        turn_check=1;
//    }
//    else turn_check=0;


    // 未找到有效点时返回0
//    return (current_min != -1.0f) ? 1 : 0;
//    return (valid_blobs > 0) ? 1 : 0;
      return 1;
}
/***************************************************************
* 函数名称：void Seek_Beacon()
* 函数输入：无
* 函数输出：无
* 功能说明：龙邱找灯思路
***************************************************************/
//底下的部分没压缩
uint8_t dothang[200];
uint8_t dotlie[200];
uint8_t aroundTime = 0; // 记录未检测到白点的时间计数器
uint8_t aroundFlag = 0; // 标志位，指示当前转向方向（例如：0为右转，1为左转）
uint8_t amendFlag = 0; // 标志位，用于判断是否需要调整转向策略
Car_Price_Type Car_Price = {0, 0, 0};
void Seek_Beacon()
{
    uint8_t tm = 0;
    uint8_t nr = 0; // 行
    uint8_t nc = 0; // 列
    uint8_t dotcnt = 0;

    int sumlie = 0, sumhang = 0;

    // 扫描图像区域，寻找连续的白点（可能是信标灯）
    for (nr = 1; nr < MT9V03X_H / 2 - 1; nr++) // 从上往下扫描
    {
        for (nc = 1; nc < MT9V03X_W / 2 - 1; nc++) // 从左往右扫描
        {
            // 如果当前像素上下左右至少有两个是白点，则认为是可信的信标点
            if ((image_bin[nr - 1][nc] + image_bin[nr + 1][nc] +
                 image_bin[nr][nc + 1] + image_bin[nr][nc - 1]) > 1)
            {
                dothang[dotcnt] = nr;         // 记录垂直方向位置（行）
                dotlie[dotcnt++] = nc;        // 记录水平方向位置（列）
            }
        }
    }

    if (dotcnt) // 如果检测到可信的白点
    {
        sumlie = 0;
        sumhang = 0;

        // 对所有检测到的白点取平均值，确定中心位置
        for (tm = 0; tm < dotcnt; tm++)
        {
            sumlie += dotlie[tm]; // 水平方向总和
            sumhang += dothang[tm]; // 垂直方向总和（暂未使用）
        }

        sumlie = sumlie / dotcnt; // 求水平中心点
        sumhang = sumhang / dotcnt; // 求垂直中心点（暂未使用）

        aroundTime = 0; // 重置转向计时器
    }
    else // 未检测到可信白点
    {
        if (aroundFlag == 0)
        {
            if (++aroundTime > 30)
            {
                Car_Price.Ture_Error = 60; // 设定固定偏转误差
                amendFlag = 0;
            }
            else
            {
                Car_Price.Ture_Error = 0; // 不做调整
            }
        }
        else
        {
            if (++aroundTime > 30)
            {
                Car_Price.Ture_Error = -60; // 反向偏转
                amendFlag = 0;
            }
            else
            {
                Car_Price.Ture_Error = 0; // 不做调整
            }
        }
    }

    if (dotcnt) // 如果确实检测到了信标灯
    {
        if (Car_Price.speed == 0) // 当前车速为0，不做出反应
        {
            Car_Price.Ture_Error = 0;
            Car_Price.Ture_Error_V = 0;
        }
        else // 车辆有速度
        {
            // 如果太近了，触发修正动作
            if (sumhang > 100 && sumhang < 120 && amendFlag == 0)
            {
                amendFlag = 1;
                if (aroundFlag == 1)
                    aroundFlag = 0;
                else
                    aroundFlag = 1;
            }

            // 如果太远了，根据中线调整方向
            if (sumhang < 40)
            {
                Car_Price.Ture_Error = MT9V03X_W / 2 / 2 - sumlie; // 与中线的差值
            }
            else // 远近适中，根据绕行标志选择偏移方向
            {
                if (aroundFlag == 0)
                {
                    Car_Price.Ture_Error = (MT9V03X_W / 2 - 30) / 2 - sumlie; // 中线偏左
                }
                else
                {
                    Car_Price.Ture_Error = (MT9V03X_W / 2 + 30) / 2 - sumlie; // 中线偏右
                }
            }
        }
    }

    return;
}


uint8_t beacon_only[MT9V03X_H][MT9V03X_W] = {{0}};


// 快速判断一个blob是否属于三圆干扰
int is_part_of_three_circles(int blob_idx) 
{  
    int center_y = blobs[blob_idx].sum_y / blobs[blob_idx].pixel_count;   // 整数质心y
    
    int pixel_cnt = blobs[blob_idx].pixel_count;

    if (center_y > 60 && pixel_cnt < 23) return 1;
    if (center_y > 45 && center_y <= 60 && pixel_cnt < 12) return 1;
    return 0;

//    float cx = (float)blobs[blob_idx].sum_x / blobs[blob_idx].pixel_count;
//    float cy = (float)blobs[blob_idx].sum_y / blobs[blob_idx].pixel_count;
//
//    // 找最近的另外两个blob
//    int nearest[2] = {-1, -1};
//    float min_dist[2] = {9999, 9999};
//
//    for (int i = 0; i < blob_count; i++) {
//        if (i == blob_idx) continue;   
//        if (blobs[i].pixel_count > 15) continue;
//        if (blobs[blob_idx].sum_y/blobs[blob_idx].pixel_count < 60) continue;
//        float xi = (float)blobs[i].sum_x / blobs[i].pixel_count;
//        float yi = (float)blobs[i].sum_y / blobs[i].pixel_count;
//        float dist = sqrt((cx-xi)*(cx-xi) + (cy-yi)*(cy-yi));
//
//        if (dist < min_dist[0])
//        {
//            min_dist[1] = min_dist[0];
//            nearest[1] = nearest[0];
//            min_dist[0] = dist;
//            nearest[0] = i;
//        } else if (dist < min_dist[1]) {
//            min_dist[1] = dist;
//            nearest[1] = i;
//        }
//    }

    // 如果最近的两个都在10像素内，可能是三圆之一
//    if (nearest[0] != -1 && nearest[1] != -1  ) {
//        if (min_dist[0] < 40 && min_dist[1] < 40 && blobs[nearest[0]].pixel_count<50 && blobs[nearest[1]].pixel_count<50) {
//            // 检查这三个点是否构成三角形
//            return 1;
//        }
//    }
//    
//    // 排除双圆
//    if(nearest[0] != - 1 )
//    {
////        float x2 = (float)blobs[nearest[0]].sum_x / blobs[nearest[0]].pixel_count;
////        float y2 = (float)blobs[nearest[0]].sum_y / blobs[nearest[0]].pixel_count;
////        float dist_pair = min_dist[0];
//        if(min_dist[0] < 40 && blobs[nearest[0]].pixel_count<80)
//        {
//           
//            return 1;
//        }
//    }

 
}

// 使用
void find_real_beacons(float *out_x, float *out_y) {
    // 1. 清空显示缓冲区
    memset(beacon_only, 0, sizeof(beacon_only));

    // 2. 标记哪些 blob 是候选信标
    uint8_t is_candidate[MAX_LABELS] = {0};
    int candidate_count = 0;

    for (int i = 0; i < blob_count; i++) {
      
        // 筛选条件：面积 ≥ 5 且 不属于三个圆（即不是干扰）
        if (blobs[i].pixel_count >= 5 && !is_part_of_three_circles(i) ) {
            is_candidate[i] = 1;
            candidate_count++;
       
           for (int y = 0; y < MT9V03X_H; y++) {
                for (int x = 0; x < MT9V03X_W; x++) {
                    if (labels[y][x] != 0) {
                        uint8_t root = parent[labels[y][x]];
                        if (root == 0) continue;
                        uint8_t blob_idx = root_map[root] - 1;  // 使用 root_map 得到正确的 blob 索引
                        if (is_candidate[blob_idx]) {
                            beacon_only[y][x] = 255;
                        }
                    }
                }
            }
        }
    }

    // 3. 如果没有候选 blob，则返回无效坐标
    if (candidate_count == 0) {
        if (out_x) *out_x = -1.0f;
        if (out_y) *out_y = -1.0f;
        return;
    }

    // 4. 在候选 blob 中找出距离 (94,100) 最近的点
    float min_dist_sq = 1e9f;
    float best_x = -1.0f, best_y = -1.0f;

    for (int i = 0; i < blob_count; i++) {
        if (!is_candidate[i]) continue;

        Blob *b = &blobs[i];
        float cx = (float)b->sum_x / b->pixel_count;
        float cy = (float)b->sum_y / b->pixel_count;
        float dx = cx - 94.0f;
        float dy = cy - 100.0f;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) 
        {
            min_dist_sq = dist_sq;
            best_x = cx;
            best_y = cy;
        }
    }

    if (out_x) *out_x = best_x;
    if (out_y) *out_y = best_y;
}

int find_car(int blob_idx)
{
    for (int i = 0; i < blob_count; i++)
    {
      float width = (float)(blobs[blob_idx].max_x - blobs[blob_idx].min_x);
      float height = (float)(blobs[blob_idx].max_y - blobs[blob_idx].min_y);
      if (height == 0)  return 0;       
      else
      {
         float rate = width / height;   
         if (rate >= 2) return 1;     
      }          
    }
    return 0;
}
