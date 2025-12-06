#include <stdio.h>

typedef enum {
    GPIO_Speed_2MHz,
    GPIO_Speed_10MHz,
    GPIO_Speed_50MHz
} Mode;  

typedef struct {
    Mode GPIO_Speed;  
} Speed;

void GPIO_StructureInit(Speed *p);

void GPIO_StructureInit(Speed *p) {
    p->GPIO_Speed = GPIO_Speed_2MHz;  
}

int main(void) {
    Speed x;  
    GPIO_StructureInit(&x);
    return 0;
}
