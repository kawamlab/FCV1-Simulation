# FCV1-Simulation
DC3のストーンシミュレーションをマルチスレッドにより、高速で行うためのリポジトリです.
(下の表は同じseconds_per_frameで行っております)
Ryzen 7 5800U 16GB
![simulation_speed](figure/Comparison_of%20speed.png)

## Release から使う
このシミュレータは、Python から呼び出す拡張モジュールとして利用します。Release から取得した `simulator.so` または `simulator.pyd` を Python から import して使用してください。

`StoneSimulator()` は初期化時に `config.json` を相対パスで読み込み、同時に OpenMP のスレッドも作成します。`data.json` を使うサンプルコードも相対パスでファイルを読み込むため、実行時は必要なファイルを同じ作業ディレクトリに置くか、ファイルパスを明示的に指定してください。

### 最短の使い方
Release 配布物をスクリプトと同じディレクトリに置く場合の最小例です。

```python
import json
import numpy as np
from simulator import StoneSimulator

stone_simulator = StoneSimulator()

with open("data.json", "r", encoding="utf-8") as read_file:
    data = json.load(read_file)

team0_position = np.array(data["team0_positions"], dtype=np.float64)
team1_position = np.array(data["team1_positions"], dtype=np.float64)
shot = data["shot"]
shot_per_team = data["shot_per_team"]
team_id = data["team_id"]
x_velocities = np.array(data["x_velocities"], dtype=np.float64)
y_velocities = np.array(data["y_velocities"], dtype=np.float64)
angular_velocities = np.array(data["angular_velocities"], dtype=np.int32)

simulated_stones_position = stone_simulator.simulator(
    team0_position,
    team1_position,
    shot,
    x_velocities,
    y_velocities,
    angular_velocities,
    team_id,
    shot_per_team,
    1,
)

print(simulated_stones_position.shape)
```

リポジトリ内のサンプルをそのまま使う場合は、`src/test.py` の通り `from build.simulator import StoneSimulator` として読み込み、`src` ディレクトリで実行してください。

## `StoneSimulator` の使い方
`StoneSimulator()` を呼ぶと、設定ファイルの読み込みと OpenMP の初期化が行われます。試合開始前の準備時間中に一度だけインスタンス化し、その後は同じインスタンスを使い回す想定です。

`simulator(...)` の主な引数は次の通りです。

1. `team0_stone_positions`
   `team0` のストーン座標です。`(8, 2)` 形式で、各要素は `[x, y]` です。
2. `team1_stone_positions`
   `team1` のストーン座標です。こちらも `(8, 2)` 形式です。
3. `total_shot`
   現在のショット数です。想定範囲は `0` から `15` です。
4. `x_velocities`
   候補ショットごとの x 方向初速度を並べた 1 次元配列です。
5. `y_velocities`
   候補ショットごとの y 方向初速度を並べた 1 次元配列です。
6. `angular_velocities`
   回転方向を並べた 1 次元配列です。`1` が `cw`、`-1` が `ccw` です。
7. `team_id`
   投球チームの番号です。`team0 -> 0`、`team1 -> 1` を指定します。
8. `shot_per_team`
   投球チーム内でのショット番号です。想定範囲は `0` から `7` です。
9. `hummer_team`
   通常利用では `1` を指定してください。この値はデバッグ用に先攻後攻の確認で使われており、通常のシミュレーションでは `1` を使う前提です。

`x_velocities`、`y_velocities`、`angular_velocities` は同じ長さにしてください。各インデックスが 1 つの候補ショットに対応し、戻り値もそのインデックス順に並びます。

## 戻り値
戻り値は `numpy.ndarray` です。shape は次の通りです。

```python
(simulation_count, 2, 8, 2)
```

各軸の意味は次の通りです。

- `simulation_count`: 候補ショット数
- `2`: `team0` と `team1`
- `8`: 各チームのストーン数
- `2`: 各ストーンの座標 `(x, y)`

つまり、各候補ショットに対して「両チーム 16 個のストーンがシミュレーション後にどこへあるか」をまとめて受け取れます。

## 設定ファイルと実行時の注意
スレッド数は `src/config.json` と同じ形式の `config.json` で設定します。

```json
{
    "thread_num": 8
}
```

注意点は次の通りです。

- `StoneSimulator()` は実行時のカレントディレクトリから `config.json` を読み込みます。
- サンプルコードは `data.json` も相対パスで読み込みます。
- Release 配布物を使う場合は、`simulator.so` または `simulator.pyd`、`config.json`、実行スクリプトを同じディレクトリに置く運用が分かりやすいです。
- リポジトリ内のサンプル `src/test.py` を使う場合は、`src/build/simulator.so`、`src/config.json`、`src/data.json` がそろった状態で `src` ディレクトリから実行してください。

## 依存関係
Release 配布物を使う場合の実行時依存関係は次の通りです。

- Python >= 3.9
- numpy < 2.0

ソースからビルドする場合は、上記に加えて `pybind11`、CMake、OpenMP を利用できる C++ コンパイラが必要です。

## ソースからビルドする方法
通常の利用では Release 配布物を使う想定ですが、開発者向けにビルド手順も載せておきます。

### Setup submodule
```bash
git submodule update --init --recursive
```

### Build Box2d
```bash
cd extern/box2d
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=ON -DCMAKE_INSTALL_PREFIX="./" ..
cmake --build .
cmake --build . --target install
```

### Build simulator
```bash
cd src/build
cmake ..
make
```

ビルド後、リポジトリ内のサンプルは次のように実行できます。

```bash
cd src
python3 test.py
```
