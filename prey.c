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

const int prey_INF = 30;          // エッジが存在しない場合の表現(無限大の距離)
int prey_adj[NODE_NUM][NODE_NUM]; // 隣接行列本体
int prey_dp[NODE_NUM][NODE_NUM];  // 最短経路


/*
 * 指定したオブジェクトの座標を取得する
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int target:PreyもしくはPredatorを表す数値
 * 返り値:targetの位置座標を表すpursuit_position構造体
 */
pursuit_position preyGetPosition
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
int preyGetNodeNum(int x, int y)
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
int preySearchNode(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int target)
{
  int returnNodeNum;
  pursuit_position targetPosition;

  targetPosition = preyGetPosition(battleFieldArray, target);
  returnNodeNum = preyGetNodeNum(targetPosition.x, targetPosition.y);

  return returnNodeNum;
}


/*
 * 格子世界データから隣接行列を生成する関数
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int adjacent[NODE_NUM][NODE_NUM]
 *      :隣接行列を表すNODE_NUM*NODE_NUMの整数型2次元配列
 */
void preyCreateAdjMatrix(int battleFieldArray[WORLD_SIZE][WORLD_SIZE],
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
void preyCopyMatrix(int copy[NODE_NUM][NODE_NUM],
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
void preyCalcShortestPath(int dist[NODE_NUM][NODE_NUM])
{
  for (int i = 0; i < NODE_NUM; i++) {
    for(int j = 0; j < NODE_NUM; j++) {
      if (dist[i][j] != 1) {        // エッジが存在しないなら
         if (i == j) continue;      // 自身への距離は0
         dist[i][j] = prey_INF;     // 障害物との距離は無限
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
 * Predatorからの距離を正規化し、評価マップを生成
 * float evalMap[WORLD_SIZE][WORLD_SIZE]
 *        :各座標における評価値を保存するWORLD_SIZE*WORLD_SIZEのfloat型二次元配列
 * int predatorNodeNum:Predatorのノード番号
 */
void preyCreateEvalMapDistance(float evalMap[WORLD_SIZE][WORLD_SIZE], int predatorNodeNum)
{
  int fromNodeNum = 0;  // ノード番号は0から走査
  float evaluation;     // 評価値
  for (int i = 0; i < WORLD_SIZE; i++) {
    for (int j = 0; j < WORLD_SIZE; j++) {
       /*
       * Predatorまでの距離を正規化
       * prey_dp[fromNodeNum][predatorNodeNum]:Predatorまでの距離
       * 評価値は距離0で0.0
       */
      if (prey_dp[fromNodeNum][predatorNodeNum] == prey_INF) {
        evaluation = 0.0;
      } else {
        evaluation = (float)(prey_dp[fromNodeNum][predatorNodeNum]) / prey_INF;
       }
      evalMap[i][j] = evaluation;
      fromNodeNum++;
     }
  }
}

/*
 * 格子世界の角隅となる場所に負の評価づけをし、評価マップを生成
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * float evalMap[WORLD_SIZE][WORLD_SIZE]
 *        :各座標における評価値を保存するWORLD_SIZE*WORLD_SIZEのfloat型二次元配列
 */
void preyCreateEvalMapCorner(int battleFieldArray[WORLD_SIZE][WORLD_SIZE],
                             float evalMap[WORLD_SIZE][WORLD_SIZE])
{
  int i,j;

  /* すべて0で初期化*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      evalMap[i][j] = 0.0;
    }
  }

  /* 各座標への評価づけ*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      if (battleFieldArray[i][j] == VALUE_OF_OBSTACLE) {
          /* 障害物は最低値*/
         evalMap[i][j] = 10.0;
      } else {
          /* 右上*/
         if (j + 1 >= WORLD_SIZE && i - 1 < 0) {
           evalMap[i][j] = 3.0;
           evalMap[i + 1][j] = 2.0;
           evalMap[i][j - 1] = 2.0;
          }
          /* 右下*/
         if (j + 1 >= WORLD_SIZE && i + 1 >= WORLD_SIZE) {
           evalMap[i][j] = 3.0;
           evalMap[i - 1][j] = 2.0;
           evalMap[i][j - 1] = 2.0;
          }
          /* 左下*/
         if (j - 1 < 0 && i + 1 >= WORLD_SIZE) {
           evalMap[i][j] = 3.0;
           evalMap[i - 1][j] = 2.0;
           evalMap[i][j + 1] = 2.0;
          }
          /* 左上*/
         if (j - 1 < 0 && i - 1 < 0) {
           evalMap[i][j] = 3.0;
           evalMap[i + 1][j] = 2.0;
           evalMap[i][j + 1] = 2.0;
          }
       }
    }
  }

  /* 値を反転して正規化*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      evalMap[i][j] = 1.0 - evalMap[i][j] / 10.0;
    }
  }
}

/*
 * 障害物に囲まれる場所に負の評価づけをし、評価マップを生成
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      :格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * float evalMap[WORLD_SIZE][WORLD_SIZE]
 *        :各座標における評価値を保存するWORLD_SIZE*WORLD_SIZEのfloat型二次元配列
 */
void preyCreateEvalMapSurround(int battleFieldArray[WORLD_SIZE][WORLD_SIZE],
                               float evalMap[WORLD_SIZE][WORLD_SIZE])
{
  int i, j;

  /* すべて0で初期化*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      evalMap[i][j] = 0.0;
    }
  }

  /* 障害物に囲まれた場所とその周辺に重み付け*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
     if (battleFieldArray[i][j] == VALUE_OF_OBSTACLE) continue;

      /*上と右の確認*/
      if (battleFieldArray[i - 1][j] == VALUE_OF_OBSTACLE
           && battleFieldArray[i][j + 1] == VALUE_OF_OBSTACLE) {
      evalMap[i][j] += 2.0;
      evalMap[i + 1][j] += 1.0;
      evalMap[i][j - 1] += 1.0;
       }
      /*下と右の確認*/
      if (battleFieldArray[i + 1][j] == VALUE_OF_OBSTACLE
           && battleFieldArray[i][j + 1] == VALUE_OF_OBSTACLE) {
      evalMap[i][j] += 2.0;
      evalMap[i - 1][j] += 1.0;
      evalMap[i][j - 1] += 1.0;
       }
      /*上と左の確認*/
      if (battleFieldArray[i - 1][j] == VALUE_OF_OBSTACLE
           && battleFieldArray[i][j - 1] == VALUE_OF_OBSTACLE) {
      evalMap[i][j] += 2.0;
      evalMap[i + 1][j] += 1.0;
      evalMap[i][j + 1] += 1.0;
       }
      /*下と左の確認*/
      if (battleFieldArray[i + 1][j] == VALUE_OF_OBSTACLE
           && battleFieldArray[i][j - 1] == VALUE_OF_OBSTACLE) {
      evalMap[i][j] += 2.0;
      evalMap[i - 1][j] += 1.0;
      evalMap[i][j + 1] += 1.0;
       }
     }
  }

  /* 障害物の評価を最低値にする*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      if (battleFieldArray[i][j] == VALUE_OF_OBSTACLE)
       evalMap[i][j] = 10.0;
    }
  }

  /* 値を反転して正規化*/
  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      evalMap[i][j] = 1.0 - evalMap[i][j] / 10.0;
    }
  }
}

/*
 * 3つのマップを積和合成する
 * float resultMap[WORLD_SIZE][WORLD_SIZE]
 *        :合成したマップを格納するWORLD_SIZE*WORLD_SIZEのfloat型二次元配列
 * float map_a[WORLD_SIZE][WORLD_SIZE]
 * float map_b[WORLD_SIZE][WORLD_SIZE]
 * float map_c[WORLD_SIZE][WORLD_SIZE]
 *        :合成するマップ
 */
void preySumOfProducts(float resultMap[WORLD_SIZE][WORLD_SIZE],
                       float map_a[WORLD_SIZE][WORLD_SIZE],
                       float map_b[WORLD_SIZE][WORLD_SIZE],
                       float map_c[WORLD_SIZE][WORLD_SIZE])
{
  int i, j;
  const float a_ratio = 0.5;  // マップAの影響比率
  const float b_ratio = 0.1;  // マップBの影響比率
  const float c_ratio = 0.4;  // マップBの影響比率

  for (i = 0; i < WORLD_SIZE; i++) {
    for (j = 0; j < WORLD_SIZE; j++) {
      resultMap[i][j] = map_a[i][j] * a_ratio + map_b[i][j] * b_ratio + map_c[i][j] * c_ratio;
    }
  }
}

/*
 * 各関数の制御及び動作の決定
 */
void Prey(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int *action)
{
  int i;
  float actionArray[4];                  // 次の行動('u','d','r','l')それぞれに対する評価値の配列
  int actionIndex = 0;                   // actionArrayの添字を格納する
  float evalMap_dist[WORLD_SIZE][WORLD_SIZE];  // Predatorからの距離の評価マップ
  float evalMap_corn[WORLD_SIZE][WORLD_SIZE];  // 格子世界の四隅の評価マップ
  float evalMap_surr[WORLD_SIZE][WORLD_SIZE];  // 障害物に囲まれた場所の評価マップ
  float evalMap_SOP[WORLD_SIZE][WORLD_SIZE];   // すべての評価マップの積和を保存するマップ, Preyが参照する
  float evalMax;                         // 次に取れる行動の中で最も評価の高い値
  pursuit_position preyPosition;         // Predator自身の座標

  /* 隣接行列の生成(初回呼び出し時のみ) */
#ifndef PRED_CREATE_ADJ
#define PRED_CREATE_ADJ
  preyCreateAdjMatrix(battleFieldArray, prey_adj);
#endif

 /* 隣接行列のコピーおよび最短距離の計算(初回呼び出し時のみ)*/
#ifndef CALC_PRED_SHORTEST
#define CALC_PRED_SHORTEST
  preyCopyMatrix(prey_dp, prey_adj);
  preyCalcShortestPath(prey_dp);
#endif

  /*評価値マップの生成*/
  preyCreateEvalMapDistance(evalMap_dist, preySearchNode(battleFieldArray, VALUE_OF_PREDATOR));
  preyCreateEvalMapCorner(battleFieldArray, evalMap_corn);
  preyCreateEvalMapSurround(battleFieldArray, evalMap_surr);
  preySumOfProducts(evalMap_SOP, evalMap_dist, evalMap_corn, evalMap_surr);

  /* 自身の現在位置を取得*/
  preyPosition = preyGetPosition(battleFieldArray, VALUE_OF_PREY);

  /*
   * 移動可能な座標の評価値配列を設定
   * Preyにとって止まるメリットはないので's'を考慮しない
   */
  for (i = 0; i < 4; i++)
    actionArray[i] = 0.0;

  /* upの評価値*/
  if (preyPosition.y - 1 >= 0)
    actionArray[0] = evalMap_SOP[preyPosition.y - 1][preyPosition.x];
  /* downの評価値*/
  if (preyPosition.y + 1 < WORLD_SIZE)
    actionArray[1] = evalMap_SOP[preyPosition.y + 1][preyPosition.x];
  /* rightの評価値*/
  if (preyPosition.x + 1 < WORLD_SIZE)
    actionArray[2] = evalMap_SOP[preyPosition.y][preyPosition.x + 1];
  /* leftの評価値*/
  if (preyPosition.x - 1 >= 0)
    actionArray[3] = evalMap_SOP[preyPosition.y][preyPosition.x - 1];

  /* 移動可能な範囲の評価値の最大値を求める*/
  evalMax = actionArray[0];
  for (i = 1; i < 4; i++) {
    if (evalMax < actionArray[i]) {
      evalMax = actionArray[i];
      actionIndex = i;
    }
  }

/*動作確認用*/
/*  for (i = 0; i < 4; i++) {
    printf("actionArray[%d] : %f\n", i, actionArray[i]);
  }
*/

  /* 評価値に基づいて動作を決定する*/
  switch(actionIndex) {
      break;
    case 0:
      *action = 'u';
      break;
    case 1:
      *action = 'd';
      break;
    case 2:
      *action = 'r';
      break;
    case 3:
      *action = 'l';
      break;
    default:
      break;
  }

  /* 動作確認用*/
  //printf("\nmax evaluation : %f, next action %c\n", evalMax, *action);
}
