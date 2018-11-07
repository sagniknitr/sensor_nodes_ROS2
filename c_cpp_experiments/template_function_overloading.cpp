#include <iostream>
using namespace std;
template <class X> void func(X a)
{
   // Function code;
   cout <<"Inside f(X a) \n";
}

template <class X, class Y> void func(X a, X b) //overloading function template func()
{
   // Function code;
   cout <<"Inside f(X a, Y b) \n";
}

int main()
{
   func<int>(10); // calls func(X a)
   func<int, int>(10, 20); // calls func(X a, Y b)
   return 0; 
}
