
#include<iostream>
#include<vector>


class a 
{
    public:
    int i = 0;

    int setValue( int j)
    {
        return j;
    }
    void getValue()
    {
        int b = setValue(2);
        std::cout << b << std::endl;
    }
};

int main()
{
    a A;
    A.setValue(10);
    A.getValue();
    return 0 ;
    std::vector<int> b ; 
    b.clear();



}