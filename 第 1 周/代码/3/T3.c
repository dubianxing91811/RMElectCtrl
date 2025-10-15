#include <stdio.h>
typedef enum {
    GPIO_Speed_2MHz,
    GPIO_Speed_10MHz,
    GPIO_Speed_50MHz
} GPIO_Speed_X;  

typedef struct {
    GPIO_Speed_X GPIO_Speed; 
} GPIO_InitX;  

void GPIO_StructureInit(GPIO_InitX* GPIO_InitX_Pt);

int main() {
    GPIO_InitX GPIO_Speed_Type;
    GPIO_StructureInit(&GPIO_Speed_Type);

    return 0;
}

void GPIO_StructureInit(GPIO_InitX* GPIO_InitX_Pt) {
    if (GPIO_InitX_Pt != NULL) {  
        GPIO_InitX_Pt->GPIO_Speed = GPIO_Speed_2MHz;     
    }
}