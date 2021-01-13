# value_iteration
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