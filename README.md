# icp_lidar_ceres
point to point icp와 point to plane 알고리즘을 최적화 라이브러리 ceres를 사용하여 구현한 C++ 패키지입니다. [Cyrill Stachniss 교수님의 ICP 강의](https://www.youtube.com/watch?v=QWDM4cFdKrE&t=33s)의 파이썬 예제를 참고하여 작성했습니다.

## Dependencies
- [Eigen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)  
- [Matplotlib C++](https://github.com/lava/matplotlib-cpp)  
- [knncpp kd tree Library](https://github.com/Rookfighter/knn-cpp)  
- [Ceres Library](https://github.com/ceres-solver/ceres-solver)  
위의 4가지 라이브러리를 `make install` 한 후에 컴파일 및 실행이 가능합니다.

## 예제 실행 방법
```
cd icp_lidar_ceres
mkdir build
cd build && cmake ..
make
./icp_lidar_ceres ../example/true_data.txt ../example/moved_data.txt
```
build 폴더를 생성하여 make 한 후 실행 파일을 실행합니다. 실행 파일의 파라미터는 각각 기준 포인트들의 파일명, icp 알고리즘에 의해 정렬될 포인트들(input points)의 파일명입니다. 두 포인트 예제 파일은 [링크](https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb#Overview)의 파이썬 예제에 의해 생성된 랜덤 포인트를 저장한 것입니다.

## 예제 실행 결과
파란색 점: 기준 points  
파란색 *: input points  
빨간색 *: icp 알고리즘에 의해 정렬된 points  
- reference_points.txt와 points_to_be_aligned.txt의 정렬 결과
<p align="center"><img src="/figs/result_1.png"></p>

- true_data.txt와 moved_data.txt의 정렬 결과
<p align="center"><img src="/figs/result_2.png"></p>

## CMakeLists.txt의 추가 설명
해당 패키지는 ICP 알고리즘의 실행 이외에도 ROS Wrapper 패키지 작성에 사용할 수 있도록 catkin 환경에 컴파일하는 기능을 갖고 있습니다. `catkin build` 명령어를 통해 빌드할 시 catkin 환경을 기준으로 컴파일 및 빌드되고, 위의 예제 실행 방법의 명령어를 통해 make 하면 단순 C++ 컴파일 및 빌드한 것과 같은 결과를 얻을 수 있습니다.