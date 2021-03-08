#include <sys/time.h>
#include <stdio.h>

static time_t seconds;
static time_t seconds_2;

int main(int argc, char const *argv[])
{
    seconds =time(NULL);
    printf("%ld\n",seconds);
    for(int i=0;i<2000000000;i++){}
    seconds_2=time(NULL);
    printf("sss\n");
    printf("%ld",seconds-seconds_2);
    return 0;
}
