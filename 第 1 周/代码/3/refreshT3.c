#include<stdio.h>

enum mode{
    GPIO_Speed_2MHz = 0,
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_50MHz = 2
};
struct speed{
    enum mode GPIO_Speed;     
};
void GPIO_StructureInit(struct speed *pData)
{
    pData->GPIO_Speed = GPIO_Speed_2MHz;   // 默认值
}
int main()
{
    struct speed x;
    struct speed *pDate = &x;
    GPIO_StructureInit(pDate);   
    
return 0;
}