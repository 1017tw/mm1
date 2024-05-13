#include<stdio.h>
#include"add.h"
#include"multi.h"
#include"sub.h"
    int main(){
        int a=1;
        int b=2;
        int c=0;
        printf("a+b=%d\n",add(a,b));
     printf("a-b=%d\n",sub(a,b));
        printf("a*b=%d\n",multi(a,b));
        return 0;
    }