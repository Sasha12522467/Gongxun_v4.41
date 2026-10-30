/* Extracted production functions; simulated camera/turntable, no hardware. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

uint8_t WuLiaoData, move_maix, yanse1, yanse2;
uint8_t material_last;
volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count, maixcam_last_update_ms;
volatile uint8_t mat_1, mat_2, mat_3;
static uint8_t task[3], ring[3], initial[3], reference_color;
static unsigned grabs, modes, phase, record_polls, wait_polls[2], rotations;
static unsigned reference_position, old_reference_position, second_position, third_position, stows;
static unsigned layout_index;
static bool query_enabled, initial_query_enabled, instant_arrival;
static uint32_t now_ms;
static VisualPosition_Result_t visual_result;
static const uint8_t color_modes[7] = {0,3,7,5,4,6,8};
static const uint8_t layouts[6][3] = {
    {1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}
};

uint32_t HAL_GetTick(void) { return now_ms; }
static uint16_t X(unsigned index)
{
    return index==0?RAW_GRAB_LEFT_X_PX:index==1?
        (phase>=3?RAW_GRAB_EXTENDED_MIDDLE_X_PX:RAW_GRAB_MIDDLE_X_PX):RAW_GRAB_RIGHT_X_PX;
}
static uint16_t Y(unsigned index)
{
    return index==0?RAW_GRAB_LEFT_Y_PX:index==1?
        (phase>=3?RAW_GRAB_EXTENDED_MIDDLE_Y_PX:RAW_GRAB_MIDDLE_Y_PX):RAW_GRAB_RIGHT_Y_PX;
}
static unsigned Find(const uint8_t values[3], uint8_t color)
{
    unsigned i;
    for(i=0;i<3;++i) if(values[i]==color) return i;
    assert(0); return 0;
}
static void Rotate(void)
{
    uint8_t copy[3]; unsigned i;
    memcpy(copy,ring,sizeof(copy));
    for(i=0;i<3;++i) ring[(i+RAW_GRAB_ROTATION_STEP)%3]=copy[i];
}
static void Arrive(void)
{
    unsigned i;
    for(i=0;i<3 && ring[reference_position]!=reference_color;++i) Rotate();
    assert(ring[reference_position]==reference_color);
}
static void Frame(uint8_t color, uint16_t x, uint16_t y)
{
    WuLiaoData=color; YuanData[0]=x; YuanData[1]=y; move_maix=0xB1;
    maixcam_last_update_ms=now_ms; ++maixcam_frame_count;
}
void osDelay(uint32_t ms)
{
    now_ms+=ms;
    if(phase==1 && ms==RAW_GRAB_POLL_MS) {
        uint8_t foreign=1;
        while(foreign==task[0] || foreign==task[1] || foreign==task[2]) ++foreign;
        ++record_polls;
        if(record_polls==1) Frame(0,X(1),Y(1));
        else if(record_polls==2) Frame(foreign,X(1),Y(1));
        else if(record_polls==3 || record_polls==4)
            Frame(initial[(1+RAW_GRAB_ROTATION_STEP)%3],X(1),Y(1));
        else if(record_polls==5) Frame(0,X(1),Y(1));
        else { assert(record_polls==6); Frame(initial[1],X(1),Y(1)); }
    } else if((phase==3 || phase==4) && ms==RAW_GRAB_POLL_MS) {
        unsigned *polls=&wait_polls[phase-3];
        unsigned tolerance=RAW_GRAB_TOLERANCE_PX;
        assert(grabs==phase-2 && (phase==4 || reference_position!=RAW_GRAB_MIDDLE));
        assert(reference_color!=task[0]);
        if(phase==4) assert(reference_color!=task[1]);
        ++*polls; assert(*polls<=8);
        if(instant_arrival) {
            /* Only the NEW state is ever visible after grabbing. Do not wait for old state. */
            assert(*polls==1); Arrive();
            Frame(reference_color,X(reference_position),Y(reference_position));
            return;
        }
        if(*polls==1) {
            /* An old-side point alone must NOT trigger next grab. Hidden old center has no frame. */
            if(phase==4 && reference_position==RAW_GRAB_MIDDLE)
                Frame(reference_color,RAW_GRAB_MIDDLE_X_PX,RAW_GRAB_MIDDLE_Y_PX); /* Initial (69,101) is not extended middle. */
            else if(old_reference_position!=RAW_GRAB_MIDDLE)
                Frame(reference_color,X(old_reference_position),Y(old_reference_position));
            else Frame(0,0,0);
            return;
        }
        Frame(reference_color,X(reference_position),Y(reference_position));
        if(*polls==2) maixcam_frame_count=0;
        if(*polls==3) maixcam_last_update_ms=now_ms-RAW_GRAB_FRAME_TIMEOUT_MS-1;
        if(*polls==4) WuLiaoData=task[0];
        if(*polls==5) YuanData[0]+=(uint16_t)(tolerance+1);
        if(*polls==6) YuanData[1]+=(uint16_t)(tolerance+1);
        if(*polls==7) move_maix=0xB2;
        if(*polls==8) {
            int delta=rotations==0?(int)tolerance:rotations==1?-(int)tolerance:0;
            int x=(int)X(reference_position)+delta;
            int y=(int)Y(reference_position)+delta;
            YuanData[0]=(uint16_t)(x<0?0:x); YuanData[1]=(uint16_t)(y<0?0:y);
            maixcam_last_update_ms=now_ms-RAW_GRAB_FRAME_TIMEOUT_MS;
            Arrive();
        }
    }
}
void ser_set(int angle, int time)
{
    if(angle==-69) assert(time==800 && grabs==0);
    else { assert(angle==-145 && time==1000 && material_last==0U); ++stows; phase=5; }
}
void ZDT_set(int address, int pulse) { assert(address==6 && pulse==1300); }
void Maixcam_setmode(uint8_t mode)
{
    ++modes;
    if(modes==1) { assert(mode==1); phase=1; return; }
    if(modes==2) {
        assert(mode==color_modes[initial[1]] && grabs==0);
        assert(yanse1==initial[(1+RAW_GRAB_ROTATION_STEP)%3]);
        assert(yanse2==initial[1]); phase=2; return;
    }
    if(modes==3) {
        assert(grabs==1);
        second_position=(Find(initial,task[1])+RAW_GRAB_ROTATION_STEP)%3;
        reference_color=second_position==RAW_GRAB_MIDDLE?task[2]:task[1];
        old_reference_position=Find(initial,reference_color);
        reference_position=(old_reference_position+RAW_GRAB_ROTATION_STEP)%3;
        phase=3;
    } else {
        assert(modes==4 && grabs==2);
        reference_color=task[2];
        old_reference_position=(Find(initial,task[2])+RAW_GRAB_ROTATION_STEP)%3;
        /* Independent expected third-grab table: only step2 layouts312/321 changed. */
#if RAW_GRAB_ROTATION_STEP == 2U
        static const unsigned third_points[6]={0U,2U,0U,2U,1U,1U};
#else
        static const unsigned third_points[6]={2U,0U,2U,0U,2U,2U};
#endif
        third_position=third_points[layout_index];
        reference_position=third_position; phase=4;
    }
    assert((phase==4 || reference_position!=RAW_GRAB_MIDDLE) && reference_position!=old_reference_position);
    assert(mode==color_modes[reference_color]); /* Hidden b must immediately select c, never old b mode. */
    Frame(reference_color,X(reference_position),Y(reference_position)); /* Buffered frame, must wait for new frame. */
}
VisualPosition_Result_t VisualPosition_RunRawMaterial(uint16_t x, uint16_t y,
    float yaw, uint8_t tolerance, uint32_t timeout)
{
    assert(x==RAW_GRAB_MIDDLE_X_PX && y==RAW_GRAB_MIDDLE_Y_PX);
    assert(yaw==RAW_GRAB_POSITION_YAW_DEG && tolerance==RAW_GRAB_POSITION_TOLERANCE_PX);
    assert(timeout==RAW_GRAB_POSITION_TIMEOUT_MS && phase==2 && record_polls==6);
    Frame(yanse2,x,y); return visual_result;
}
void ChassisRuntime_SetEncoderQueryEnabled(bool enabled) { query_enabled=enabled; }
bool ChassisRuntime_IsEncoderQueryEnabled(void) { return query_enabled; }
void RawGrab_GrabPosition(RawGrab_Position_t position, uint8_t slot)
{
    unsigned i;
    assert(!query_enabled && grabs<3 && slot==grabs+1);
    assert(material_last == (slot == 3U ? 1U : 0U));
    assert(ring[position]==task[grabs]);
    if(grabs==0) assert(position==Find(initial,task[0]) && modes==2);
    else if(grabs==1) {
        assert((unsigned)position==second_position && modes==3);
        assert(wait_polls[0]==(instant_arrival?1U:8U));
        if(position==RAW_GRAB_MIDDLE) assert(reference_color==task[2]);
        assert(mat_1==task[0] && mat_2==0);
    } else {
        assert((unsigned)position==third_position && modes==4);
#if RAW_GRAB_ROTATION_STEP == 2U
        if(layout_index>=4U) {
            assert(position==RAW_GRAB_MIDDLE && X(position)==70U && Y(position)==148U);
        } else assert(position!=RAW_GRAB_MIDDLE);
#endif
        assert(wait_polls[1]==(instant_arrival?1U:8U));
        assert(mat_1==task[0] && mat_2==task[1] && mat_3==0);
    }
    ring[position]=0; ++grabs; now_ms+=4000;
    for(i=0;i<rotations;++i) Rotate();
}

/* INSERT_ACTUAL_PRODUCTION_FUNCTIONS */

static void Reset(uint8_t a, uint8_t b, uint8_t c, unsigned layout, unsigned turn)
{
    unsigned i;
    task[0]=a; task[1]=b; task[2]=c;
    layout_index=layout;
    for(i=0;i<3;++i) ring[i]=initial[i]=task[layouts[layout][i]-1];
    grabs=modes=phase=record_polls=stows=0;
    memset(wait_polls,0,sizeof(wait_polls));
    rotations=turn; query_enabled=initial_query_enabled=true; instant_arrival=false;
    material_last=1U; /* The production entry must clear a leftover flag. */
    visual_result=VISUAL_POSITION_OK; now_ms=turn==2?UINT32_MAX-200:1000;
    maixcam_frame_count=42; maixcam_last_update_ms=now_ms;
    WuLiaoData=initial[1]; move_maix=0xB1;
    mat_1=mat_2=mat_3=6;
}
int main(void)
{
    uint8_t a,b,c;
    unsigned layout,turn,query,instant,cases=0;
    for(a=1;a<=6;++a) {
        assert(ColorToCameraMode(a)==color_modes[a]);
        for(b=1;b<=6;++b) for(c=1;c<=6;++c) {
            if(a==b || a==c || b==c) continue;
            for(layout=0;layout<6;++layout) for(turn=0;turn<3;++turn)
            for(query=0;query<2;++query) for(instant=0;instant<2;++instant) {
                Reset(a,b,c,layout,turn);
                query_enabled=initial_query_enabled=query!=0; instant_arrival=instant!=0;
                assert(RawGrab_Run(a,b,c)==1);
                assert(grabs==3 && modes==4 && stows==1);
                assert(material_last==0U);
                assert(query_enabled==initial_query_enabled);
                assert(mat_1==a && mat_2==b && mat_3==c);
                assert(ring[0]==0 && ring[1]==0 && ring[2]==0);
                assert(raw_grab_test_layout==100*layouts[layout][0]+10*layouts[layout][1]+layouts[layout][2]);
                ++cases;
            }
        }
    }
    Reset(1,5,6,0,0); assert(!RawGrab_Run(0,5,6) && !grabs && !modes && stows==1);
    Reset(1,5,6,0,0); assert(!RawGrab_Run(1,1,6) && !grabs && !modes && stows==1);
    Reset(1,5,6,0,0); assert(!RawGrab_Run(1,5,7) && !grabs && !modes && stows==1);
    Reset(1,5,6,4,0); visual_result=VISUAL_POSITION_TIMEOUT;
    assert(!RawGrab_Run(1,5,6) && !grabs && query_enabled && stows==1);
    Reset(1,5,6,4,0); visual_result=VISUAL_POSITION_BAD_PARAMETER;
    assert(!RawGrab_Run(1,5,6) && !grabs && query_enabled && stows==1);
    printf("PASS: step=%u, %u cases; 312/321 step2 third grab at extended middle70/148; other layouts and reverse rotation preserved; initial69/101 rejected for extended middle, stale/color/command/boundary checks retained.\n",
        RAW_GRAB_ROTATION_STEP,cases);
    return 0;
}
