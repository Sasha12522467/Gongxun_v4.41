/* Host-only verification of extracted production actuator functions. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

typedef struct { int kind,a,b; } Event;
static Event events[512], expected[512];
static unsigned count,delay_ms;

void PutMaterialFromSlot(int number,uint8_t slot);
void PutStorageMaterialFromSlot(int number,uint8_t slot);
void PutMaterial2FromSlot(int number,uint8_t slot);
void GrabMaterialToSlot(int number,uint8_t slot);
void Processing_First(uint8_t a,uint8_t b,uint8_t c);
void Processing_Second(uint8_t a,uint8_t b,uint8_t c);
void Storage_First(uint8_t a,uint8_t b,uint8_t c);
void Storage_Second(uint8_t a,uint8_t b,uint8_t c);

static void Record(int kind,int a,int b) {
    assert(count<512U);
    events[count].kind=kind; events[count].a=a; events[count].b=b; ++count;
}
void ZDT_set(int id,int pulse) { Record(1,id,pulse); }
void ZDT_set2(int id,int pulse) { Record(7,id,pulse); }
void HAL_Delay(unsigned ms) { Record(2,(int)ms,0); delay_ms+=ms; }
void osDelay(unsigned ms) { Record(3,(int)ms,0); delay_ms+=ms; }
void ser_set(int angle,int duration) { Record(4,angle,duration); }
void Plate(int slot) { Record(5,slot,0); }
void servo_set(int state) { Record(6,state,0); }
static void SameEvents(unsigned n) {
    assert(count==n);
    for(unsigned i=0;i<n;++i)
        assert(events[i].kind==expected[i].kind && events[i].a==expected[i].a && events[i].b==expected[i].b);
}
static void SaveEvents(void) {
    for(unsigned i=0;i<count;++i) expected[i]=events[i];
}
static unsigned TailDelay(uint8_t ring) { return ring==1U?600U:ring==3U?500U:0U; }

static void TestLeafFlags(void) {
    void (*place[3])(int,uint8_t)={PutMaterialFromSlot,PutStorageMaterialFromSlot,PutMaterial2FromSlot};
    for(unsigned layer=0;layer<3;++layer) for(uint8_t ring=1;ring<=3;++ring)
    for(uint8_t slot=1;slot<=3;++slot) {
        unsigned n,normal_delay,removed=ring==2U?0U:2U;
        count=delay_ms=0U; material_last=0U; place[layer](ring,slot);
        n=count; normal_delay=delay_ms; SaveEvents();
        if(removed) {
            assert(expected[n-2].kind==4 && expected[n-2].a==-70);
            assert(expected[n-1].kind==2 && expected[n-1].a==(int)TailDelay(ring));
        }
        count=delay_ms=0U; material_last=1U; place[layer](ring,slot);
        SameEvents(n-removed);
        assert(delay_ms+TailDelay(ring)==normal_delay);
    }
    puts("PASS: processing, independent storage layer1 and storage layer2 x3 rings x3 slots; last-action flag and lift/claw/pulse commands preserved.");
}

static void TestRetrievalFlags(void) {
    for(uint8_t ring=1;ring<=3;++ring) for(uint8_t slot=1;slot<=3;++slot) {
        unsigned n,normal_delay,motor_calls=0U;
        count=delay_ms=0U; material_last=0U; GrabMaterialToSlot(ring,slot);
        n=count; normal_delay=delay_ms; SaveEvents();
        for(unsigned i=0;i<n;++i) {
            assert(expected[i].kind!=1); /* Retrieval must select ZDT_set2. */
            if(expected[i].kind==7) ++motor_calls;
        }
        assert(motor_calls==8U);
        assert(expected[n-2].kind==4 && expected[n-2].a==-69 && expected[n-2].b==800);
        assert(expected[n-1].kind==2 && expected[n-1].a==500);
        count=delay_ms=0U; material_last=1U; GrabMaterialToSlot(ring,slot);
        SameEvents(n-2U);
        assert(delay_ms+500U==normal_delay);
    }
    puts("PASS: retrieval x3 rings x3 slots uses separate motor profile; final flag removes only middle-turn and 500ms; all lift/claw/pulse commands preserved.");
}

static unsigned BuildExpected(unsigned group,const uint8_t order[3]) {
    void (*place)(int,uint8_t)=group==3U?PutMaterial2FromSlot:
        group==2U?PutStorageMaterialFromSlot:PutMaterialFromSlot;
    unsigned removed;
    count=delay_ms=0U; material_last=0U;
    for(uint8_t i=0;i<3;++i) place(order[i],i+1U);
    removed=order[2]==2U?0U:2U;
    if(removed) {
        assert(events[count-2].kind==4 && events[count-2].a==-70);
        assert(events[count-1].kind==2 && events[count-1].a==(int)TailDelay(order[2]));
        count-=removed; delay_ms-=TailDelay(order[2]);
    }
    if(group<2U) {
        for(uint8_t i=0;i<3;++i) GrabMaterialToSlot(order[i],i+1U);
        assert(events[count-2].kind==4 && events[count-2].a==-69);
        assert(events[count-1].kind==2 && events[count-1].a==500);
        count-=2U; delay_ms-=500U;
        ZDT_set(1,0); osDelay(100);
    }
    SaveEvents(); return count;
}
static void TestGroups(void) {
    static const uint8_t permutations[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    void (*groups[4])(uint8_t,uint8_t,uint8_t)={Processing_First,Processing_Second,Storage_First,Storage_Second};
    unsigned cases=0U;
    for(unsigned group=0;group<4;++group) for(unsigned p=0;p<6;++p)
    for(uint8_t initial_flag=0;initial_flag<=1;++initial_flag) {
        const uint8_t *order=permutations[p];
        unsigned n=BuildExpected(group,order),expected_delay=delay_ms;
        count=delay_ms=0U; material_last=initial_flag;
        groups[group](order[0],order[1],order[2]);
        SameEvents(n);
        assert(delay_ms==expected_delay && material_last==0U);
        ++cases;
    }
    printf("PASS: %u group/permutation/initial-flag cases; first2 placements/retrievals normal, third skips, actual ring order respected, flag reset.\n",cases);
}
int main(void) {
    TestLeafFlags(); TestRetrievalFlags(); TestGroups(); return 0;
}
