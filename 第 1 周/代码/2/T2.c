#include<stdio.h>
int main()
{
int x=0;
scanf("%d",&x);
while(x!=-1)
{
    if (x==0)
    {
        printf("helloworld");
    }else if (x==1)
    {
        printf("HELLOWORLD");
    }
    scanf("%d",&x);
}    
    return 0;
}