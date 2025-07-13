# AStar Algorithms
このレポジトリでは、様々なA*アルゴリズムのアニメーションを簡単に紹介します。visualizationにはncursesを利用しています。

This repository explains various A* algorithm implementations with animations.

## Installation & Usage
devcontainerを利用しているので、vscode内でReopen in containerで仮想環境を立ち上げてください。他デフォルトのc++環境でも動きます。(ncursesが一応必要)

Set up the virtual environment using ```Reopen in container``` in your vscode. You may build and run the program in the default c++ environment. (ncurses required)

```bash
# build
make

# build dir clean
make clean

# dijkstra
./astars dijkstra

# native A*
./astars astar

# weighted A*
./astars weighted_astar

# bidirectional A*
./astars bidirectional_astar

# theta*
./astars thetastar
```
