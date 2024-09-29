<!--
 * @Author: Xiawenlong-bug 2473833028@qq.com
 * @Date: 2024-08-29 16:02:54
 * @LastEditors: Xiawenlong-bug 2473833028@qq.com
 * @LastEditTime: 2024-08-29 17:10:23
 * @FilePath: /hw2_ws/readme.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->

```shell
source devel/setup.bash
roslaunch grid_path_searcher demo.launch
```

实现A*算法，其中启发函数是Euclidean
实现JPS算法

主函数在demo_node.cpp
具体使用哪个算法，需要在demo_node.cpp里选择
```c++
//_use_jps = 0 -> Do not use JPS
//_use_jps = 1 -> Use JPS
//you just need to change the #define value of _use_jps
#define _use_jps 1
#if _use_jps
```



