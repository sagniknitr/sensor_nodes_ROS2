#include <stdio.h>
#include <string.h>
union unionJob
{
   //defining a union
   string s1;
   float salary;
   int workerNo;
} uJob;

struct structJob
{
   string s2;
   float salary;
   int workerNo;
} sJob;

int main()
{
   printf("size of union = %d", sizeof(uJob));
   printf("\nsize of structure = %d", sizeof(sJob));
   return 0;
}
