#ifndef CHASSIS_DIRECTION_H
#define CHASSIS_DIRECTION_H

/* 四轮电机连同轮子对角互换补偿，只改这个开关，重新编译、烧录并复位。
 * 0：正常装配，逻辑轮m0/m1/m2/m3对应驱动器地址2/3/4/5。
 * 1：两组对角均互换，即2号与4号互换、3号与5号互换；地址变为4/5/2/3。
 * 适用前提：驱动器地址仍跟随原电机，未同时改地址或重新接线抵消互换。
 * 速度输出、粗转脉冲、视觉微调和编码器回读必须共用下面的地址映射；
 * 导航内部轮序、编码器正负号和前后/侧向标定不因地址互换而重复反向。
 * 此开关不处理单独翻装电机、麦轮滚子左右型号装错等其它机械问题。
 * 不允许行驶中切换；复位后重新建立编码器基准，先架空低速核对方向。
 */
#ifndef CHASSIS_DIAGONAL_SWAP
#define CHASSIS_DIAGONAL_SWAP 1
#endif

#if CHASSIS_DIAGONAL_SWAP == 0
#define CHASSIS_MOTOR_M0_ADDRESS 2U
#define CHASSIS_MOTOR_M1_ADDRESS 3U
#define CHASSIS_MOTOR_M2_ADDRESS 4U
#define CHASSIS_MOTOR_M3_ADDRESS 5U
#elif CHASSIS_DIAGONAL_SWAP == 1
#define CHASSIS_MOTOR_M0_ADDRESS 4U
#define CHASSIS_MOTOR_M1_ADDRESS 5U
#define CHASSIS_MOTOR_M2_ADDRESS 2U
#define CHASSIS_MOTOR_M3_ADDRESS 3U
#else
#error CHASSIS_DIAGONAL_SWAP_must_be_zero_or_one
#endif

/* 仅用于数组初始化，顺序始终是逻辑轮m0/m1/m2/m3，不是地址升序。 */
#define CHASSIS_MOTOR_ADDRESS_LIST {CHASSIS_MOTOR_M0_ADDRESS, \
    CHASSIS_MOTOR_M1_ADDRESS, CHASSIS_MOTOR_M2_ADDRESS, CHASSIS_MOTOR_M3_ADDRESS}

/*
 * 麦轮机体平移方向校准，必须同时用于轮速正解和编码器逆解。
 * 前进是Nav的vx/视觉全向控制的x；侧向是Nav的vy/视觉的y。
 * 当前实车反馈：原前进命令实际后退，因此只反前后通道。
 * 不反转全部轮速，不改变旋转、粗转脉冲方向或编码器协议符号。
 * 后续前后方向正确时保持-1；要恢复旧解算只把前后值改为1。
 * 旧MotorSetVel/MotorSetPulse与独立odometer_bench不使用此校准，
 * 它们目前没有正式调用；若以后启用，需独立核对方向。
 */
#ifndef CHASSIS_FORWARD_SIGN
#define CHASSIS_FORWARD_SIGN (-1)
#endif

#ifndef CHASSIS_LATERAL_SIGN
#define CHASSIS_LATERAL_SIGN 1
#endif

#if (CHASSIS_FORWARD_SIGN != 1) && (CHASSIS_FORWARD_SIGN != -1)
#error CHASSIS_FORWARD_SIGN_must_be_plus_or_minus_one
#endif
#if (CHASSIS_LATERAL_SIGN != 1) && (CHASSIS_LATERAL_SIGN != -1)
#error CHASSIS_LATERAL_SIGN_must_be_plus_or_minus_one
#endif

#endif /* CHASSIS_DIRECTION_H */
