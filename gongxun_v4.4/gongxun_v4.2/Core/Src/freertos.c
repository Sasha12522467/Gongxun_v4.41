/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "chassis_runtime.h"
#include "path_follower.h"
#include "radar_startup.h"
#include "robot_task.h"
#include "action.h"
#include "QR_code.h"
#include <math.h>
#include "tim.h"
#include "actionInit.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 雷达只在主流程启动阶段扫描一次，区域由两个启动键同步选择。 */
#define RADAR_STARTUP_NODE_ID       1U
#define RADAR_STARTUP_TIMEOUT_MS    20000U
#define RADAR_STARTUP_POLL_MS       5U
#define START_KEY_DEBOUNCE_MS       20U
#define START_FEEDBACK_TIMEOUT_MS  5000U
#define START_POSE_TIMEOUT_MS      1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
AppStart_State_t g_app_start;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t MainTask_RunStageAction(uint8_t completed_stage);
static uint8_t MainTask_PrepareStart(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  (void)ChassisRuntime_Create();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 512);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

//雷达的启动扫描和规划***********************************************************************
  /* 先选择实际放车区域并建立地图位姿，不能先用1区路线规划再选2区。 */
  if (MainTask_PrepareStart() != 0U)
  {
    g_app_start.status = APP_START_PLANNING;
  /*
   * 启动阶段只扫描一次全场障碍并完成A*规划。
   * 结果保存在g_radar_startup.path，成功后再进入正式比赛主流程。
   */
#if APP_RADAR_ENABLED
  /* 接回雷达时使用：握手、选区、扫描障碍、A*规划。 */
  (void)RadarStartup_Start(RADAR_STARTUP_NODE_ID,
                          g_app_start.selected_zone,
                          RADAR_STARTUP_TIMEOUT_MS);
  while (!RadarStartup_IsFinished())
  {
    RadarStartup_Process();
    osDelay(RADAR_STARTUP_POLL_MS);
  }
#else
  /* 当前未接雷达：直接按无新增障碍做A*，不等待20秒通信超时。
   * 不调用RadarClient_Init/Poll，不发送雷达握手或扫描指令。
   */
  (void)RadarStartup_PlanWithoutRadar();
#endif

  if (RadarStartup_IsSuccessful() && PathFollower_Start(RadarStartup_GetPath()))
  {
    /* 两种模式都使用同一个路径执行器，工位动作和离站流程不变。 */
    g_app_start.status = APP_START_RUNNING;
  }
  else
  {
    g_app_start.fault = RadarStartup_IsSuccessful() ?
                        APP_START_FAULT_FOLLOWER : APP_START_FAULT_PLAN;
    g_app_start.status = APP_START_ERROR;
    (void)ChassisRuntime_Stop();
  }
  }
		 
//雷达的启动扫描和规划************************************************************************	
//测试代码************************************************************************************


 
  
  
  
//	 QRcode_print_1();
//	/* 先启动MaixCam物料模式，并等待第一批有效坐标帧。 */
//	osDelay(100U);
//		Maixcam_setmode(5U);
//		(void)VisualPosition_Run(155U, 138, 90.0f, 1U, 8000U); //加工区，地图航向
//  	(void)VisualPosition_Run(78U, 93, 0.0f, 1U, 8000U);   //码垛
	//	(void)VisualPosition_Run(60U, 100, -90.0f, 1U, 8000U);   //原料区，地图航向
//	    ser_set(-69, 800);
//    osDelay(810U);
//	/* 视觉微调直接控制四轮，期间关闭任务2，避免USART3指令互相覆盖。 */
//	ChassisRuntime_SetEncoderQueryEnabled(false);
//	MotorSetPulse(5000, 1, 200, 200);
//	osDelay(1000);
//		put_material2(3);
//		put_material2(3);
//	put_material2(3);
////	osDelay(20U);
//	grab_material(3);
//	grab_material(2);
//	grab_material(3);
//	put_material(1);
//	put_material(2);
//	put_material(3);
//	grab_middle(1);
//	ChassisRuntime_SetEncoderQueryEnabled(true);
//VisualPosition_RunRawMaterial(RAW_GRAB_MIDDLE_X_PX, RAW_GRAB_MIDDLE_Y_PX,
//            0, RAW_GRAB_POSITION_TOLERANCE_PX,
//            RAW_GRAB_POSITION_TIMEOUT_MS);
//VisualPosition_Run(145U, 127U, 0.0f, 1U, 8000U);  //加工区定位点
//VisualPosition_Run(72U, 83U, 0.0f, 1U, 8000U);  //码垛定位点
//ChassisRuntime_SetEncoderQueryEnabled(false);
//    PutMaterial2FromSlot(1, 1U);
//    PutMaterial2FromSlot(2, 2U);
//    PutMaterial2FromSlot(3, 3U);

//    GrabMaterialToSlot(1, 1U);
//   GrabMaterialToSlot(2, 2U);
//    GrabMaterialToSlot(3, 3U);

//   RawGrab_Run(1,2,3);
//ChassisRuntime_SetEncoderQueryEnabled(true);
//  while(1)
//	{
//		osDelay(10);
//	}
//测试代码************************************************************************************
  /* Infinite loop：联合执行A*路径，电机闭环仍由高优先级任务负责。 */
  for(;;)
  {
    if (g_app_start.status == APP_START_RUNNING)
    {
      PathFollower_Process();
      /*
       * 车辆到达一个任务段终点并完全停车后，才进入对应工位处理。
       * MainTask_RunStageAction返回1表示机构已收回、共享串口动作结束。
       * 原料/加工/暂存区由PathFollower_Continue先自动全向离站，再转向下一段。
       */
      if (g_path_follower.status == PATH_FOLLOWER_WAIT_ACTION)
      {
        if (MainTask_RunStageAction(g_path_follower.completed_stage) != 0U)
        {
          (void)PathFollower_Continue();
        }
      }
      if (g_path_follower.status == PATH_FOLLOWER_FAULT)
      {
        g_app_start.fault = APP_START_FAULT_FOLLOWER;
        g_app_start.status = APP_START_ERROR;
      }
      else if (g_path_follower.status == PATH_FOLLOWER_DONE)
      {
        g_app_start.status = APP_START_DONE;
      }
    }
    else
    {
      /* 雷达或规划失败：当前保持停车，等待后续故障处理。 */
    }
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
__weak void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint32_t last_wake = osKernelSysTick();
  (void)argument;

  /* 20 ms固定周期：四轮回读 -> 里程计/IMU融合 -> 四轮速度输出。 */
  for(;;)
  {
    ChassisRuntime_TaskStep();
    osDelayUntil(&last_wake,20U);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 两键同时按下不选择，避免放在2区却误选1区；只读取一次并锁定。 */
static uint8_t MainTask_ReadStartKey(void)
{
    GPIO_PinState key1 = HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin);
    GPIO_PinState key2 = HAL_GPIO_ReadPin(START_KEY2_GPIO_Port, START_KEY2_Pin);
    if (key1 == GPIO_PIN_RESET && key2 == GPIO_PIN_SET) return PP_START_ZONE_1;
    if (key2 == GPIO_PIN_RESET && key1 == GPIO_PIN_SET) return PP_START_ZONE_2;
    return 0U;
}

/* 建立选区位姿，只修正软件坐标，不发送移动目标。
 * 高优先级任务2继续运行，每次等待都osDelay让出CPU，严禁用while空转。
 * 队列接口返回成功只是入队，必须等对应序号真正完成且位姿确认才启动。
 */
static uint8_t MainTask_PrepareStart(void)
{
    const PathPlan_StartProfile_t *profile;
    ChassisRuntime_State_t chassis;
    uint32_t wait_start;
    uint8_t zone;
    float yaw_error;

    g_app_start.status = APP_START_WAIT_KEY;
    g_app_start.fault = APP_START_FAULT_NONE;
    g_app_start.selected_zone = 0U;
    g_app_start.pose_sequence = 0U;
    for (;;) {
        zone = MainTask_ReadStartKey();
        if (zone != 0U) {
            osDelay(START_KEY_DEBOUNCE_MS);
            if (MainTask_ReadStartKey() == zone) break;
        }
        osDelay(5U);
    }
    if (path_plan_select_start_zone(zone) == 0) {
        g_app_start.fault = APP_START_FAULT_BAD_ZONE;
        goto start_failed;
    }
    g_app_start.selected_zone = zone;
    profile = path_plan_get_start_profile();
    g_app_start.status = APP_START_WAIT_FEEDBACK;
    wait_start = HAL_GetTick();
    for (;;) {
        ChassisRuntime_GetState(&chassis);
        if (chassis.status == CHASSIS_RUNTIME_FAULT) {
            g_app_start.fault = APP_START_FAULT_CHASSIS;
            goto start_failed;
        }
        /* IMU有效后把当前原始读数对齐PP_START_YAW_DEG配置的地图航向。
         * 不要求IMU开机读数为0；先按配置角保持航向离站，再允许路径转向。
         */
        if (chassis.nav_initialized != 0U && chassis.encoder_query_enabled != 0U &&
            chassis.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
            chassis.imu_valid != 0U && isfinite(chassis.imu_yaw_deg) &&
            (chassis.status == CHASSIS_RUNTIME_IDLE ||
             chassis.status == CHASSIS_RUNTIME_DONE)) break;
        if (HAL_GetTick() - wait_start >= START_FEEDBACK_TIMEOUT_MS) {
            g_app_start.fault = APP_START_FAULT_FEEDBACK_TIMEOUT;
            goto start_failed;
        }
        osDelay(5U);
    }
    if (!ChassisRuntime_CorrectPoseWithSequence(profile->initial_x_mm,
                                                profile->initial_y_mm,
                                                profile->initial_yaw_deg,
                                                &g_app_start.pose_sequence)) {
        g_app_start.fault = APP_START_FAULT_POSE_QUEUE;
        goto start_failed;
    }
    g_app_start.status = APP_START_WAIT_POSE;
    wait_start = HAL_GetTick();
    for (;;) {
        ChassisRuntime_GetState(&chassis);
        if (chassis.status == CHASSIS_RUNTIME_FAULT) {
            g_app_start.fault = APP_START_FAULT_CHASSIS;
            goto start_failed;
        }
        yaw_error = chassis.nav.pose.yaw_deg - profile->initial_yaw_deg;
        if (isfinite(yaw_error)) {
            while (yaw_error > 180.0f) yaw_error -= 360.0f;
            while (yaw_error < -180.0f) yaw_error += 360.0f;
        }
        if (g_app_start.pose_sequence != 0U &&
            chassis.completed_sequence == g_app_start.pose_sequence &&
            (chassis.status == CHASSIS_RUNTIME_IDLE ||
             chassis.status == CHASSIS_RUNTIME_DONE) &&
            chassis.encoder_valid_mask == CHASSIS_NAV_ENCODER_ALL_VALID &&
            isfinite(chassis.nav.pose.x_mm) && isfinite(chassis.nav.pose.y_mm) &&
            isfinite(yaw_error) && fabsf(yaw_error) <= 1.0f &&
            hypotf(chassis.nav.pose.x_mm - profile->initial_x_mm,
                   chassis.nav.pose.y_mm - profile->initial_y_mm) <= 2.0f) return 1U;
        if (HAL_GetTick() - wait_start >= START_POSE_TIMEOUT_MS) {
            g_app_start.fault = APP_START_FAULT_POSE_TIMEOUT;
            goto start_failed;
        }
        osDelay(5U);
    }

start_failed:
    g_app_start.status = APP_START_ERROR;
    (void)ChassisRuntime_Stop();
    return 0U;
}

/*
 * 工位右侧对接姿态的临时观察时间。
 * 正式运行设为0；需要观察对接方向时可临时改为1000毫秒。
 * 动作本身的完成状态决定何时继续路径。
 */
/* 正式动作已接入，取消旧的工位对接后额外1秒观察等待。 */
#define STAGE_DOCK_TEST_HOLD_MS 0U

/*
 * 六次工位动作各自的执行顺序，下标与completed_stage相同。
 * 每行依次是a、b、c；修改某行只影响那一次到站，不影响另一圈。
 * 原料行是颜色编号1~6：第一圈来自A1/B1/C1，第二圈来自A3/B3/C3。
 * 第一圈加工/暂存用A2/B2/C2；第二圈加工仍用A4/B4/C4。
 * 第二圈暂存按第1/2/3组扫码计算同色目标，不使用第4组加工顺序。
 * 原料按抓取先后存入槽1/2/3；加工、暂存参数a/b/c分别指定这三个槽
 * 应去的圆环位置。例：原料6/2/4、圆环3/1/2，则槽1颜色6放圆环3，
 * 槽2颜色2放圆环1，槽3颜色4放圆环2；加工取回时仍回到原槽。
 * 表在实际扫码或手动任务加载后生成，不再混用A1/A2/A3，也不把颜色号当槽号。
 */
static uint8_t s_stage_action_order[PP_MAX_STAGES][3] = {
    {0U, 0U, 0U}, /* 0：扫码，无机构顺序。 */
    {0U, 0U, 0U}, /* 1：第一圈原料，扫码第1组，颜色1~6。 */
    {0U, 0U, 0U}, /* 2：第一圈加工，扫码第2组，圆环1~3。 */
    {0U, 0U, 0U}, /* 3：第一圈暂存，扫码第2组，圆环1~3。 */
    {0U, 0U, 0U}, /* 4：第二圈原料，扫码第3组，颜色1~6。 */
    {0U, 0U, 0U}, /* 5：第二圈加工，扫码第4组，圆环1~3。 */
    {0U, 0U, 0U}, /* 6：第二圈暂存，同色匹配的目标圆环，第二层叠放。 */
    {0U, 0U, 0U}  /* 7：最终停车，无机构顺序。 */
};

/**
  * @brief  执行每个任务段终点对应的扫码、视觉定位和机构动作。
  * @param  completed_stage 已完成的任务段编号，范围0~7。
  * @retval 1=本阶段动作完成，可以继续；0=动作仍在进行，保持停车等待。
  * @note
  *   1. 阻塞式函数：在对应case中直接调用，函数返回后保持return 1U。
  *   2. 非阻塞式函数：每次推进一步，未完成返回0U，完成后返回1U。
  *   3. 视觉得到绝对位姿后可调用：
  *        ChassisRuntime_CorrectPose(x_mm, y_mm, yaw_deg);
  *      该接口只校正地图位姿，不清空编码器累计值。
  *   4. 不要在这里长时间关闭中断或暂停调度；ZDT_task会继续读取里程计。
  *   5. 加工/暂存区返回1U前需将机构收回；返回后路径状态机自动恢复
  *      编码器回读，保持工位航向低速回到离站安全点，再转向下一段。
  */
static uint8_t MainTask_RunStageAction(uint8_t completed_stage)
{
  uint8_t *order;
  uint8_t max_id;
  /* 只记录两次原料到站：下标0=第一圈、1=第二圈，失败不自动重抓。
   * 1=动作函数返回成功，非传感器确认抓牢；调试排障后复位重跑。
   */
  static uint8_t raw_attempted[2];
  static uint8_t raw_done[2];

  if (completed_stage >= PP_MAX_STAGES) return 0U;
  order = s_stage_action_order[completed_stage];
  max_id = (completed_stage == 1U || completed_stage == 4U) ? 6U : 3U;

  /* 避免无效编号导致原料忙等待永不退出，或抓放函数不执行却放行路径。 */
  if (completed_stage >= 1U && completed_stage <= 6U &&
      (order[0] < 1U || order[0] > max_id ||
       order[1] < 1U || order[1] > max_id ||
       order[2] < 1U || order[2] > max_id ||
       order[0] == order[1] || order[0] == order[2] || order[1] == order[2]))
  {
    return 0U; /* 顺序配置错误：保持WAIT_ACTION，不执行机构或继续路径。 */
  }

#if STAGE_DOCK_TEST_HOLD_MS > 0U
  static uint8_t hold_stage = 0xFFU;
  static uint32_t hold_start_ms = 0U;

  /* 阶段1~6都是原料/加工/暂存工位，先非阻塞保持对接姿态供观察。 */
  if (completed_stage >= 1U && completed_stage <= 6U &&
      STAGE_DOCK_TEST_HOLD_MS != 0U)
  {
    if (hold_stage != completed_stage)
    {
      hold_stage = completed_stage;
      hold_start_ms = HAL_GetTick();
      return 0U;
    }

    if ((uint32_t)(HAL_GetTick() - hold_start_ms) <
        STAGE_DOCK_TEST_HOLD_MS)
    {
      return 0U;
    }

    hold_stage = 0xFFU;
  }
#endif

  switch (completed_stage)
  {
    case 0U:
      /* 扫码区域：车身左侧朝向二维码板，底盘已经停车。 */
      if (g_storage_stack.fault != STORAGE_STACK_FAULT_NONE) return 0U;
      raw_attempted[0] = raw_attempted[1] = 0U;
      raw_done[0] = raw_done[1] = 0U;
#if APP_MANUAL_TASK_ENABLED
      /* 手动测试：不调用扫码等待；同样解码成A1~C4，复用后续流程。
       * 不改扫码接收标志E和中断接收缓冲，避免与实际接收互相覆盖。
       */
      openmv_shun_xu1 = APP_MANUAL_TASK_GROUP1;
      openmv_shun_xu2 = APP_MANUAL_TASK_GROUP2;
      openmv_shun_xu3 = APP_MANUAL_TASK_GROUP3;
      openmv_shun_xu4 = APP_MANUAL_TASK_GROUP4;
      if (!ShunXu_Decode(openmv_shun_xu1, 6U, &A1, &B1, &C1) ||
          !ShunXu_Decode(openmv_shun_xu2, 3U, &A2, &B2, &C2) ||
          !ShunXu_Decode(openmv_shun_xu3, 6U, &A3, &B3, &C3) ||
          !ShunXu_Decode(openmv_shun_xu4, 3U, &A4, &B4, &C4)) {
        g_storage_stack.fault = STORAGE_STACK_FAULT_BAD_SCAN;
        return 0U; /* 手动编号非法时保持停车，不能盲目执行机构。 */
      }
#else
      QRcode_print_1();
#endif
      /* 第一圈：第1组颜色取料，第2组圆环加工/暂存。 */
      s_stage_action_order[1][0] = A1;
      s_stage_action_order[1][1] = B1;
      s_stage_action_order[1][2] = C1;
      s_stage_action_order[2][0] = A2;
      s_stage_action_order[2][1] = B2;
      s_stage_action_order[2][2] = C2;
      s_stage_action_order[3][0] = A2;
      s_stage_action_order[3][1] = B2;
      s_stage_action_order[3][2] = C2;

      /* 第二圈：第3组颜色取料，第4组只用于加工区。 */
      s_stage_action_order[4][0] = A3;
      s_stage_action_order[4][1] = B3;
      s_stage_action_order[4][2] = C3;
      s_stage_action_order[5][0] = A4;
      s_stage_action_order[5][1] = B4;
      s_stage_action_order[5][2] = C4;
      /* 根据扫码快照保存第一层圆环颜色，并生成第二圈同色码垛目标。
       * 两圈颜色不同/编号非法时保持停车，Watch查看g_storage_stack.fault。
       */
      if (!StorageStack_Configure(s_stage_action_order[1],
                                   s_stage_action_order[3],
                                   s_stage_action_order[4])) return 0U;
      s_stage_action_order[6][0] = g_storage_stack.target_ring[0];
      s_stage_action_order[6][1] = g_storage_stack.target_ring[1];
      s_stage_action_order[6][2] = g_storage_stack.target_ring[2];
      return 1U;
    case 1U:
      /* 第一圈原料：第1组颜色，action.c完成排列识别、定位、左中右抓取、
       * 查询开关恢复和收臂。返回0保持WAIT_ACTION，不放行路径或重复抓取。
       */
      if (raw_attempted[0] != 0U) return raw_done[0];
      raw_attempted[0] = 1U;
      raw_done[0] = RawGrab_Run(order[0], order[1], order[2]);
      return raw_done[0];

    case 4U:
      /* 第二圈原料：第3组颜色；直接调用同一流程，完成标志仍与第一圈独立。 */
      if (raw_attempted[1] != 0U) return raw_done[1];
      raw_attempted[1] = 1U;
      raw_done[1] = RawGrab_Run(order[0], order[1], order[2]);
      return raw_done[1];

    case 2U:
      /* 第一圈加工：保持原视觉模式、定位坐标及+90度工位航向。 */
		  Maixcam_setmode(2U);
			ser_set(-70,1000);
	    osDelay(700);
		  VisualPosition_Run(145U, 127U, 90.0f, 1U, 8000U);  //加工区定位点
		ChassisRuntime_SetEncoderQueryEnabled(false);
      /* 共享串口已让出，按本阶段顺序放置并取回。 */
      Processing_First(order[0], order[1], order[2]);
		ChassisRuntime_SetEncoderQueryEnabled(true);
			ser_set(-145,1000);
	    osDelay(700);
      return 1U;

    case 5U:
      /* 第二圈加工：定位与第一圈相同，动作顺序和入口独立。 */
      Maixcam_setmode(2U);
      ser_set(-70, 1000);
      osDelay(700U);
     VisualPosition_Run(145U, 127U, 90.0f, 1U, 8000U);  //加工区定位点
      ChassisRuntime_SetEncoderQueryEnabled(false);
      Processing_Second(order[0], order[1], order[2]);
      ChassisRuntime_SetEncoderQueryEnabled(true);
      /* 必须收回机构后返回1，之后路径状态机才允许离站。 */
      ser_set(-145, 1000);
      osDelay(700U);
      return 1U;

    case 3U:
      /* 第一圈暂存：原圆环模式和定位坐标不变，按第2组放第一层。
       * 定位失败不抓放、不放行；成功后记录first_layer_done。
       */
      return StorageStack_RunFirst() ? 1U : 0U;
    case 6U:
      /* 第二圈暂存：识别ring_color[2]对应物料，只定位成功才同色叠放。
       * 定位仍为(78,93,0度,1像素,8000ms)，失败锁存故障，禁止盲码垛。
       */
      return StorageStack_RunSecond() ? 1U : 0U;
    case 7U:
      /* TODO 用户代码：已经返回启停区1。
       * 可在这里执行比赛结束提示、蜂鸣器或最终停车动作。
       */
      return 1U;

    default:
      /* 非法阶段不放行，车辆保持停车，便于调试发现问题。 */
      return 0U;
  }
}

/* USER CODE END Application */
