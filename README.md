# FCV1-Simulation
DC3のストーンシミュレーションをマルチスレッドにより、高速で行うためのリポジトリです.
(下の表は同じseconds_per_frameで行っております)
Ryzen 7 5800U 16GB
![simulation_speed](figure/Comparison_of%20speed.png)

## Setup submodule
```
git submodule update --init --recursive
```

## Build Box2d
```
cd extern/box2d
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=ON -DCMAKE_INSTALL_PREFIX="./" ..
cmake --build .
cmake --build . --target install
```

## Build simulator
CmakeLists.txtの4行目は,バージョン8以上必須
```
cd /workspaces/FCV1-Simulation/src/build/
cmake ..
make
```

## Usage
### Set number of threads
srcディレクトリ内のconfig.jsonの中の
```
"thread_num": 8
```
の値で設定される.

### Notes
このプログラムを使用する際は
```
stone_simulator = StoneSimulator()
```
でStoneSimulatorクラスをインスタンス化してください.ここで,OpenMPによるスレッド作成も行うため,
試合開始前の準備時間中にインスタンス化してください.


### How to use Simulator
簡単な使用方法はtest.pyの通りである。
simulator関数の引数は
1. 先攻・後攻の順に格納したストーン座標
2. ストーンのショット数(0~15)
3. x方向の初速度ベクトル
4. y方向の初速度ベクトル
5. 回転方向(cw->1, ccw->-1)

戻り値は
1. simulator関数の引数にあるx・y方向の初速度ベクトル、回転方向のインデックス番号順に、シミュレーション後のストーン座標
2. ファイブロックルールが適用されたかどうかのbool値

## Dependencies
- numpy < 2.0
- python >= 3.9
