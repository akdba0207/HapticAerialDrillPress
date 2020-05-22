
#include <unistd.h>
#include <iostream>

int main()
{

    std::cout << "Dear Andy !!!!" << '\n';
    sleep(2);
    std::cout << "My boi Andy !!!!" << '\n';
    sleep(2);
    std::cout << "Happy birthday!!!!" << '\n';
    sleep(2);
    const char heart[] = "\xe2\x99\xa5";
    std::cout << heart << heart << heart << heart << heart << '\n';
    sleep(3);
    return 0;
}
