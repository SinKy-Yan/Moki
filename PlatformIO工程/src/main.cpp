/* 引用库 */
#include "moki.h"

/* 数字是接了几个电机 */
Moki moki(2);

void setup()
{
    moki.setup();
}

void loop()
{
    moki.run(0);
}
