	#ifndef COORDINATES_H
	#define COORDINATES_H
	#include <math.h>
	#include <float.h>
	#include <stdint.h>
	// 定义数学常量
	#define M_PI 3.14159265358979323846
	/**
	 * 查找0-90度范围内的障碍物并计算x,y坐标
	 * 支持将多个角度检测到的同一个障碍物融合为一个点
	 * @param lidar_array 激光雷达距离数组 (181个元素，类型为uint16_t)
	 * @param x1 第一个障碍物的x坐标输出
	 * @param y1 第一个障碍物的y坐标输出
	 * @param x2 第二个障碍物的x坐标输出 (可选)
	 * @param y2 第二个障碍物的y坐标输出 (可选)
	 * @return 找到的障碍物数量 (0, 1或2)
	 */
	int find_obstacles_0_to_90(const uint16_t lidar_array[181], int *x1, int *y1, float *x2, float *y2);
	#endif // COORDINATES_H

