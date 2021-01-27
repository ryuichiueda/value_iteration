# value_iteration
## How to install
1. [value_iteration](https://github.com/ryuichiueda/value_iteration/tree/master)をインストール

```
cd catkin_ws/src
git clone https://github.com/ryuichiueda/value_iteration.git
```
2. [grid_map ](https://github.com/uhobeike/grid_map.git)をインストール
```
cd catkin_ws/src
git clone -b vi_grid https://github.com/uhobeike/grid_map.git
```
3. [grid_map ](https://github.com/uhobeike/grid_map.git)の依存パッケージをインストール
```
sudo apt-get install libeigen3-dev
```
4. ビルド
```
cd catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release
or
catkin_make -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

## How to use 
10s後ぐらいにコストが表示されます。
```
roslaunch value_iteration vi.launch
```
|実行動画|
|---|
|[![](https://i9.ytimg.com/vi/315H-RMzHVA/mq2.jpg?sqp=CITuxoAG&rs=AOn4CLBOXycgv8SvK4W4aZzpP0c7QFzBQA)](http://www.youtube.com/watch?v=315H-RMzHVA)|
## How to do gtest
1. ビルド
```
catkin build value_iteration --catkin-make-args tests
```
2. テスト
```
rostest value_iteration utest_launch.test
```
以下のコマンドでも、テスト可能（出力結果が違う）
```
roslaunch value_iteration utest_launch.launch
```