#include<iostream> 
using namespace std; 
  
class BaseClass 
{ 
public: 
    virtual void Display() 
    { 
        cout << "\nThis is Display() method"
                " of BaseClass"; 
    } 
    void Show() 
    { 
        cout << "\nThis is Show() method "
               "of BaseClass"; 
    } 
}; 
  
class DerivedClass : public BaseClass 
{ 
public: 
    // Overriding method - new working of 
    // base class's display method 
    int Display() 
    { 
        cout << "\nThis is Display() method"
               " of DerivedClass"; 
        return 0;
    } 
}; 
  
// Driver code 
int main() 
{ 
    DerivedClass dr; 
    BaseClass bs = &dr; 
    bs.Display(); 
    dr.Show(); 
}
