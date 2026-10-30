	#include "Coordinates.h"
	#include <math.h>
	#include <float.h>
	#include <stdint.h>
	#define M_PI 3.14159265358979323846
	/**
	 * 查找0-90度范围内的障碍物并计算x,y坐标
	 * @param lidar_array 激光雷达距离数组 (181个元素，类型为uint16_t)
	 * @param x1 第一个障碍物的x坐标输出
	 * @param y1 第一个障碍物的y坐标输出
	 * @param x2 第二个障碍物的x坐标输出 (可选)
	 * @param y2 第二个障碍物的y坐标输出 (可选)
	 * @return 找到的障碍物数量 (0, 1或2)
	 */
	int find_obstacles_0_to_90(const uint16_t lidar_array[181], int *x1, int *y1, float *x2, float *y2) {
    // 初始化输出值为0
    *x1 = 0;
    *y1 = 0;
    if (x2) *x2 = 0.0f;
    if (y2) *y2 = 0.0f;
    
    float min_dist1 = FLT_MAX;  // 最近的障碍物距离
    float min_dist2 = FLT_MAX;  // 第二近的障碍物距离
    int count = 0;
    
    // 遍历0-90度范围(索引0-90)
    for (int i = 0; i <= 90; i++) {
        // 跳过无效值(0表示无障碍物)
        if (lidar_array[i] == 0) continue;
        
        float distance = (float)lidar_array[i];
        float angle_rad = i * M_PI / 180.0f;  // 角度转弧度
        
        // 根据您的坐标系重新计算坐标:
        // 0度: y轴正方向 (x=0, y=distance)
        // 90度: x轴正方向 (x=distance, y=0)
        // 顺时针方向，使用sin计算x，cos计算y
        float x = distance * sinf(angle_rad);
        float y = distance * cosf(angle_rad);
        
        // 检查是否比当前最近的障碍物更近
        if (distance < min_dist1) {
            // 更新第二个障碍物为原来的第一个
            min_dist2 = min_dist1;
            if (x2) *x2 = *x1;
            if (y2) *y2 = *y1;
            
            // 更新第一个障碍物
            min_dist1 = distance;
            *x1 = (int)x;
            *y1 = (int)y;
            count = (min_dist2 != FLT_MAX) ? 2 : 1;
        } 
        // 检查是否比当前第二近的障碍物更近
        else if (distance < min_dist2) {
            min_dist2 = distance;
            if (x2) *x2 = x;
            if (y2) *y2 = y;
            count = 2;
        }
    }
    
    return count;
}	
	

