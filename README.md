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
cd catkin_ws 
rosdep install -r -y --from-paths --ignore-src src/
sudo apt-get install libeigen3-dev
```
4. ビルド

何回かエラーが出るかもしれないが、構わずビルドを行うと良い。
```
cd catkin_ws
catkin build -DCMAKE_BUILD_TYPE=Release
or
catkin_make -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```
5. ros melodicユーザーの方　Python3を使えるようにする
```
(多分これ)
sudo pip3 install rospkg catkin_pkg
```

## How to use 
10s後ぐらいにコストが表示されます。
```
roslaunch value_iteration vi.launch
```
|実行動画|
|---|
|[![](https://i.gyazo.com/19c0217423e9011f125e2a8742204f5d.png)](https://www.youtube.com/watch?v=i8h4V2y6eDE&feature=youtu.be)|
## How to gtest
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