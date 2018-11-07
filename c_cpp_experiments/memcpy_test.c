/* memset example */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
int main ()
{
  char str[] = "almost every programmer should know memset!";
   int *ptr;
  ptr = (int*) malloc(sizeof(int));
   memset (ptr,111,1);
  printf("%d",*ptr);
  return 0;
}
