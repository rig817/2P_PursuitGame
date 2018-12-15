/*
 * Author:       Ryota Narita
 * Student ID:   b1014056
 * Class:        L
 * Created:      October 30, 2018
 * Language:     C
 */

/*
 * 必要なライブラリのロード
 * (stdio.hおよびstdlib.h)
 */
#ifndef STDIO_H
#define STDIO_H
#include <stdio.h>
#endif

#ifndef STDLIB_H
#define STDLIB_H
#include <stdlib.h>
#endif

/* predatorの数値表現 */
#ifndef VALUE_OF_PREDATOR
#define VALUE_OF_PREDATOR 1
#endif

/* preyの数値表現 */
#ifndef VALUE_OF_PREY
#define VALUE_OF_PREY 10
#endif

/* obstacleの数値表現 */
#ifndef VALUE_OF_OBSTACLE
#define VALUE_OF_OBSTACLE -1
#endif

/* free gridの数値表現 */
#ifndef VALUE_OF_FREE
#define VALUE_OF_FREE 0
#endif

/* 格子世界の大きさ */
#ifndef WORLD_SIZE
#define WORLD_SIZE 8
#endif

/* 隣接行列の大きさ */
#ifndef NODE_NUM
#define NODE_NUM 64
#endif

/*
 *PredatorおよびPreyの位置を表す構造体
 */
typedef struct{
  int x;
  int y;
} pursuit_position;

const int predator_INF = 30;          // エッジが存在しない場合の表現(無限大の距離)
int predator_adj[NODE_NUM][NODE_NUM]; // 隣接行列本体
int predator_dp[NODE_NUM][NODE_NUM];  // 最短経路


/*
 * 指定したオブジェクトの座標を取得する
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int target:PreyもしくはPredatorを表す数値
 * 返り値:targetの位置座標を表すpursuit_position構造体
 */
pursuit_position predatorGetPosition
(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int target)
{
  int i,j;
  pursuit_position returnPosition;

  /*
   * battleFieldArrayのなかで、値がtargetになっているところを
   * 探して、その座標をreturnPositionに代入してreturnする
   */
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      if (battleFieldArray[i][j] == target){
	returnPosition.x = j;
	returnPosition.y = i;
	return returnPosition;
      }
    }
  }
}

/*
 * 座標のノード番号を返す
 * int x:指定する座標のx座標
 * int y:指定する座標のy座標
 * 返り値:ノードの番号
 */
int predatorGetNodeNum(int x, int y)
{
  int returnNodeNum = 0;

  /* y軸はWORLD_SIZEの倍数、x軸は+座標分(0~7)*/
  returnNodeNum = (returnNodeNum + y * WORLD_SIZE) + x;
  return returnNodeNum;
}

/*
 * 指定したオブジェクトのノードを探す
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int target:PreyもしくはPredatorを表す数値
 * 返り値:targetのノード番号
 */
int predatorSearchNode(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int target)
{
  int returnNodeNum;
  pursuit_position targetPosition;

  targetPosition = predatorGetPosition(battleFieldArray, target);
  returnNodeNum = predatorGetNodeNum(targetPosition.x, targetPosition.y);

  return returnNodeNum;
}

/*
 * 格子世界データから隣接行列を生成する関数
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int adjacent[NODE_NUM][NODE_NUM]
 *      :隣接行列を表すNODE_NUM*NODE_NUMの整数型2次元配列
 */
void predatorCreateAdjMatrix(int battleFieldArray[WORLD_SIZE][WORLD_SIZE],
                             int adjacent[NODE_NUM][NODE_NUM])
{
  int i, j;
  int from = -1;      // 走査するノード, 最初のループに入ったときに0になる
  int to = from + 1;  // fromの右隣のノード

  /* 隣接行列を0で初期化*/
  for (int i = 0; i < NODE_NUM; i++) {
    for (int j = 0; j < NODE_NUM; j++) {
      adjacent[i][j] = 0;
    }
  }

  /* x軸方向のエッジを検出*/
  for (i = 0; i < WORLD_SIZE; i++) {
    /* 走査するノードを１列右にシフト*/
    from += 1;
    to = from + 1;

    for (j = 0; j < WORLD_SIZE - 1; j++) {
      if (battleFieldArray[i][j] != VALUE_OF_OBSTACLE
           && battleFieldArray[i][j+1] != VALUE_OF_OBSTACLE) {
        adjacent[from][to] = 1;
        adjacent[to][from] = 1;
      }

      from++;
      to++;
    }
  }

  from = (WORLD_SIZE * (WORLD_SIZE - 1) - 1);  // 最初のループに入ったときに0になる
  to = from + WORLD_SIZE;                      // fromの下のノード

  /* y軸方向のエッジを検出*/
  for (i = 0; i < WORLD_SIZE; i++) {
    /* 走査するノードを１列右にシフト*/
    from -= (WORLD_SIZE * (WORLD_SIZE - 1) - 1);
    to = from + WORLD_SIZE;

    for (j = 0; j < WORLD_SIZE - 1; j++) {
      if (battleFieldArray[j][i] != VALUE_OF_OBSTACLE
           && battleFieldArray[j+1][i] != VALUE_OF_OBSTACLE) {
        adjacent[from][to] = 1;
        adjacent[to][from] = 1;
      }

      from += WORLD_SIZE;
      to = from + WORLD_SIZE;
    }
  }

}

/*
 * 隣接行列のコピーを行う
 * int copy[NODE_NUM][NODE_NUM]:コピー先
 * int original[NODE_NUM][NODE_NUM]:コピー元
 */
void predatorCopyMatrix(int copy[NODE_NUM][NODE_NUM],
	      int original[NODE_NUM][NODE_NUM]){
  int i,j;

  for (i = 0; i < NODE_NUM; i++){
    for (j = 0; j < NODE_NUM; j++){
      copy[i][j] = original[i][j];
    }
  }
}

/*
 * ワーシャルフロイド法による最短距離の計算を行う
 * int dist[NODE_NUM][NODE_NUM]:
 * 各ノードからの最短距離を保持するNODE_NUM*NODE_NUMの整数型二次元配列
 */
void predatorCalcShortestPath(int dist[NODE_NUM][NODE_NUM])
{

  for (int i = 0; i < NODE_NUM; i++) {
    for(int j = 0; j < NODE_NUM; j++) {
      if (dist[i][j] != 1) {        // エッジが存在しないなら
         if (i == j) continue;      // 自身への距離は0
         dist[i][j] = predator_INF; // 障害物との距離は無限
       }
    }
  }

  /* ワーシャルフロイド法*/
  for (int k = 0; k < NODE_NUM; k++) {
    for(int i = 0; i < NODE_NUM; i++) {
      for (int j = 0; j < NODE_NUM; j++) {
         if (dist[i][j] > dist[i][k] + dist[k][j])
            dist[i][j] = dist[i][k] + dist[k][j];
       }
    }
  }
}


/*
 * Preyからの距離を正規化し、評価マップを生成
 * float evalMap[WORLD_SIZE][WORLD_SIZE]
 *        :各座標における評価値を保存するWORLD_SIZE*WORLD_SIZEのfloat型二次元配列
 * int preyNodeNum:Preyのノード番号
 */
void predatorCreateEvalMap(float evalMap[WORLD_SIZE][WORLD_SIZE], int preyNodeNum)
{
  int fromNodeNum = 0;  // ノード番号は0から走査
  float evaluation;     // 評価値
  for (int i = 0; i < WORLD_SIZE; i++) {
    for (int j = 0; j < WORLD_SIZE; j++) {
       /*
       * Preyまでの距離を正規化
       * predator_dp[fromNodeNum][preyNodeNum]:Preyまでの距離
       * 評価値は距離0で1.0, 距離INFで0.0
       */
      evaluation = (float)(predator_INF - predator_dp[fromNodeNum][preyNodeNum]) / predator_INF;
      evalMap[i][j] = evaluation;

      fromNodeNum++;
     }
  }
}

/*
 * 各関数の制御及び動作の決定
 */
void Predator(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int *action)
{
  int i;
  float actionArray[5];                  // 次の行動('s','u','d','r','l')それぞれに対する評価値の配列
  int actionIndex = 0;                   // actionArrayの添字を格納する
  float evalMap[WORLD_SIZE][WORLD_SIZE]; // Predatorが参照する評価マップ
  float evalMax;                         // 次に取れる行動の中で最も評価の高い値
  pursuit_position predatorPosition;     // Predator自身の座標

  /* 隣接行列の生成(初回呼び出し時のみ) */
#ifndef PRED_CREATE_ADJ
#define PRED_CREATE_ADJ
  predatorCreateAdjMatrix(battleFieldArray, predator_adj);
#endif

 /* 隣接行列のコピーおよび最短距離の計算(初回呼び出し時のみ)*/
#ifndef CALC_PRED_SHORTEST
#define CALC_PRED_SHORTEST
  predatorCopyMatrix(predator_dp, predator_adj);
  predatorCalcShortestPath(predator_dp);
#endif

  /*評価値マップの生成*/
  predatorCreateEvalMap(evalMap, predatorSearchNode(battleFieldArray, VALUE_OF_PREY));

  /* Predatorの現在位置を取得*/
  predatorPosition = predatorGetPosition(battleFieldArray, VALUE_OF_PREDATOR);

  /* 移動可能な座標の評価値配列を設定*/
  for (i = 0; i < 5; i++)
    actionArray[i] = 0;

  /* stayの評価値*/
  actionArray[0] = evalMap[predatorPosition.y][predatorPosition.x];
  /* upの評価値*/
  if (predatorPosition.y - 1 >= 0)
    actionArray[1] = evalMap[predatorPosition.y - 1][predatorPosition.x];
  /* downの評価値*/
  if (predatorPosition.y + 1 < WORLD_SIZE)
    actionArray[2] = evalMap[predatorPosition.y + 1][predatorPosition.x];
  /* rightの評価値*/
  if (predatorPosition.x + 1 < WORLD_SIZE)
    actionArray[3] = evalMap[predatorPosition.y][predatorPosition.x + 1];
  /* leftの評価値*/
  if (predatorPosition.x - 1 >= 0)
    actionArray[4] = evalMap[predatorPosition.y][predatorPosition.x - 1];

  /* 移動可能な範囲の評価値の最大値を求める*/
  evalMax = actionArray[0];
  for (i = 1; i < 5; i++) {
    if (evalMax < actionArray[i]) {
      evalMax = actionArray[i];
      actionIndex = i;
    }
  }

/*動作確認用*/
/*  for (i = 0; i < 5; i++) {
    printf("actionArray[%d] : %f\n", i, actionArray[i]);
  }
*/

  /* 評価値に基づいて動作を決定する*/
  switch(actionIndex) {
    case 0:
      *action = 's';
      break;
    case 1:
      *action = 'u';
      break;
    case 2:
      *action = 'd';
      break;
    case 3:
      *action = 'r';
      break;
    case 4:
      *action = 'l';
      break;
    default:
      break;
  }

  /*動作確認用*/
  //printf("\nmax evaluation : %f, next action %c\n", evalMax, *action);
}
