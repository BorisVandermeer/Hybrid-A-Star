# Hybrid A Star 的实现

[Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FBorisVandermeer&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)

一个Hybrid A Star 的基础实现。
使用多个圆描述车辆的碰撞，模拟阿克曼转向模拟车辆的运动。

用户可继承HybridAStar类，并且修改其中的虚函数以实现自己的Hybrid A Star 子类。

## 构建方式

- 仓库显示使用的是OpenCV 4 实现的。仅用于显示。

``` bash
# Setup
bash ./Setup.sh

# build
mkdir build && cd build
cmake ..
make
```

## Run A Demo

``` shell
./build/node
```

用四次点击描述规划。第一次点击是选择起点位置，第二次是起点方向。第三次是终点位置，第四次是终点方向。

效果如下：

![效果图1](docs/Pics/1.png)
![效果图2](docs/Pics/2.png)
