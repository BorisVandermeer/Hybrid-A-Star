# BorisMathLib

![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FBorisVandermeer&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)

BorisVandermeer 的一些基础数学仓库

## 主要内容

- 曲线模块：RS曲线
- 插值模块：Spline 插值，Spline曲线
- 点集模块：定义了公用的点的类型

## Run A Demo

以当前仓库为工程的顶层目录

```bash

# build 
mkdir build
cmake ..
make

# Run
./test/***

```

## How To Use

将仓库包含在工程内，使用${BORIS_MATH_LIBRARIES}、 ${BORIS_MATH_INCLUDE_DIRS}、 ${BORIS_MATH_FOUND}三个变量进行操作。
