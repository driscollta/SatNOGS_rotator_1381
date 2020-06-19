/*!
* @brief implement ftoa function following 
* https://tutorialspoint.dev/language/c/convert-floating-point-number-string
*
* @section license License
*
* Copyright (c) 2020 Tom Driscoll. 
* Based on code by Elwood Downey found on Clearskyinstitute.com and published in QEX Mar/Apr 2016.
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions: The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
* WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _UTIL_H
#define _UTIL_H  
#include<math.h> 

class Util {
    private:
    void reverse(char *str, int len);
    int intToStr(int x, char str[], int d);

    public:
    void ftoa(float n, char *res, int afterpoint);

};

#endif // _UTIL_H