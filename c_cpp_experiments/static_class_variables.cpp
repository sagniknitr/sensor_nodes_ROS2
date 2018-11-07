
// C++ program to demonstrate static 
// variables inside a class 
  
#include<iostream> 
using namespace std; 
  
class GfG 
{ 
   public: 
     static int i=0; 
      
     GfG() 
     { 
        // Do nothing 
     }; 
}; 
//int GfG::i = 1; 


int main() 
{ 
  GfG obj1; 
  GfG obj2; 
  obj1.i =2; 
  obj2.i = 3; 
    
  // prints value of i 
  cout << obj1.i<<" "<<obj2.i;    
} 
