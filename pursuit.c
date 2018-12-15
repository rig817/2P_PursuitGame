/*
 * 2 Players Pursuit Game 管理プログラム
 * version 1.2 SP2
 *
 * by E. Osawa and E. Yoshikawa, Sept 2002.
 * 
 * last revision by E. Osawa Oct. 2002
 *
 * (変更履歴)
 * 2002.10.1            障害物,自由格子,捕食者,獲物を表す定数の導入
 *
 * 2000.12.26(1.2.1)    コメント追加
 *                      threadキャンセル時のsegmentation faultの解消
 * 2000.12.25(1.2)      watch dog timerを追加．
 *                      PreyやPredatorの実行時間が1秒か監視する．
 * 2000.12.20(1.1 SP2): オプションに関するエラー処理を強化。
 *                      負の値を指定すると無限ループに陥る現象や、
 *                      オプションのみが指定されてもエラーを出さない現象、
 *                      未定義のオプションに対してエラーを出さない現象などを
 *                      改善
 *
 * 2000.12.20(1.1 SP1): PreyやPredatorの計算占有時間によって、表示の更新が
 *                      ちらつく現象を改善
 *                      デフォルトの格子世界ファイルを"battlefield.txt"から
 *                      "battlefield.dat"に変更
 * 
 * 2000.12.20(1.1)    : 表示の際にいれるウェイト(のデフォルト)を0.5秒に調整
 *                      ゲームステップ数(のデフォルト)を30に変更
 *                      ゲームステップ数を"-s 30",ウェイトの大きさを
 *                      "-t 500000",また、初期データが格納されている
 *                      "-w battlefield2.txt"のように、実行ファイルに対する
 *                      オプションとしてruntimeに読み込めるように変更
 *                      PredatorとPreyの勝敗に関するメッセージ表示を追加
 * 
 * 2000.12.19(1.0 SP2): 構造体名をposition→pursuit_positionへ変更
 *                      PreyおよびPredatorへ渡す格子世界の配列を,
 *                      そのものの参照渡し→コピーの参照渡しに変更
 *                      (PredatorやPreyが格子世界を書き換えることを防ぐため)
 *                      Preyが「3回のうちに2回だけ動ける」という仕様の解釈を
 *                      変更、「動ける確率66％」という確率的な挙動から
 *                     「2回動いて1回休む」という確定的な挙動へ
 *
 * 2000.12.15(1.0 SP1): 仕様に合致していない点を改善
 *                      (配列の縦と横が逆転していたのを修正)
 */


#define VERSION_1_2 1

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

#ifndef STRING_H
#define STRING_H
#include <string.h>
#endif

#ifdef VERSION_1_2
#ifndef PTHREAD_H
#include <pthread.h>
#endif /* PTHREAD_H */
#endif /* VERSION_1_2 */

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
 
/* 繰り返しステップ数(デフォルト値) */
#ifndef GAME_NUM
#define GAME_NUM 30
#endif

/* VALID(妥当である) */
#ifndef VALID
#define VALID 1
#endif

/* INVALID(妥当でない) */
#ifndef INVALID
#define INVALID 0
#endif

/* 格子世界ファイル名の最大文字数 */
#ifndef WORLD_FILE_NAME_LENGTH
#define WORLD_FILE_NAME_LENGTH 256
#endif

/* 表示のウェイト(デフォルト値) */
#ifdef VERSION_1_2
#ifndef DISPLAY_WAIT
#define DISPLAY_WAIT 0
#endif
#else /* VERSION_1_2 */
#ifndef DISPLAY_WAIT
#define DISPLAY_WAIT 500000
#endif
#endif /* VERSION_1_2 */

/*
 *PredatorおよびPreyの位置を表す構造体
 */
typedef struct{
  int x;
  int y;
} pursuit_position;


#ifdef VERSION_1_2

/* 
 * watch dog timer用の構体
 */
typedef struct{
  pthread_mutex_t mutex; /* 排他制御用 */
  int count;             /* watch dog timer */
  int state;             /* watch dog state */
}watchdog_t;

static watchdog_t wdtimer =
{
  PTHREAD_MUTEX_INITIALIZER,
  0,
  0
};

#define WATCHDOG_COUNT_INIT  1
#define WATCHDOG_INTERVAL 1000000 /* usec */

#define WATCHDOG_RUN           1 /* WD status */
#define WATCHDOG_STOP          0 /* WD status */

pthread_t wd_tid;     /* Watch dog スレッドのthread ID */
pthread_t player_tid; /* Prey/Predator スレッドのthread ID */

#endif /* VERSION_1_2 */


/*
 * 初期状態データファイルの読み込み
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * char *fileName: 格子世界ファイル名が格納された文字列
 * 返り値:正常終了なら0,異常終了なら1
 */
int ReadInitConf(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], char *fileName){
  FILE *fp;
  int i, j;

  fp = fopen(fileName, "r");
  if (fp == NULL){
    /* ファイルオープンエラー */
    return(1);
  }
  else{
    for (i = 0; i < WORLD_SIZE; i++){
      for (j = 0; j < WORLD_SIZE; j++){

	if (EOF == fscanf(fp,"%d", &battleFieldArray[i][j])){
	  /* ファイルの長さが想定していたよりも短いエラー */
	  return(1);
	}

      }
    }

    fclose(fp);
    return(0);
  }
}

/*
 * 格子世界を表示する関数
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int clockNum: 現在のクロック数
 * 返り値:正常終了なら0,異常終了なら1
 */
int PrintWorld(int battleFieldArray[WORLD_SIZE][WORLD_SIZE], int clockNum){
  int i, j;

  printf("clock %d\n", clockNum);
  for (i = 0; i < WORLD_SIZE; i++){
    for (j = 0; j < WORLD_SIZE; j++){

      switch(battleFieldArray[i][j]){
      case VALUE_OF_OBSTACLE: /* 障害物 */
	printf ("#");
	break;
      case VALUE_OF_FREE: /* 自由格子 */
	printf(".");
	break;
      case VALUE_OF_PREDATOR: /* Predator */
	printf("@");
	break;
      case VALUE_OF_PREY: /* Prey */
	printf("*");
	break;
      default: /* ここには本来到達しないはず */
	fprintf(stderr, "%d\n",battleFieldArray[i][j]);
	fprintf(stderr, "Execution Error!! \n");
	return(1);
	break;
      }
    }
    printf("\n");
  }
  printf("\n");
  return(0);
}

/*
 * Predatorの位置座標を取得する
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * 返り値:Predatorの位置座標を表すpursuit_position構造体
 */
pursuit_position getPredatorPosition
(int battleFieldArray[WORLD_SIZE][WORLD_SIZE]){
  
  int i,j;
  pursuit_position returnPosition;

  /*
   * battleFieldArrayのなかで、値がVALUE_OF_PREDATORになっているところを
   * 探して、その座標をreturnPositionに代入してreturnする
   */
  for (i=0; i<WORLD_SIZE; i++){
    for (j=0; j<WORLD_SIZE; j++){
      if (battleFieldArray[i][j] == VALUE_OF_PREDATOR){
	returnPosition.x=j;
	returnPosition.y=i;
	return returnPosition;
      }
    }
  }

  /*
   * ここまで処理が来るということは異常事態なので、positionとして
   * (-1,-1)を返す
   */
  returnPosition.x=-1;
  returnPosition.y=-1;
  return returnPosition;
}

/*
 * Preyの位置座標を取得する
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * 返り値:Preyの位置座標を表すpursuit_position構造体
 */
pursuit_position getPreyPosition
(int battleFieldArray[WORLD_SIZE][WORLD_SIZE]){
  
  int i,j;
  pursuit_position returnPosition;
  /*
   * battleFieldArrayのなかで、値がVALUE_OF_PREYになっているところを
   * 探して、その座標をreturnPositionに代入してreturnする
   */
  for (i=0; i<WORLD_SIZE; i++){
    for (j=0; j<WORLD_SIZE; j++){
      if (battleFieldArray[i][j] == VALUE_OF_PREY){
	returnPosition.x=j;
	returnPosition.y=i;
	return returnPosition;
      }
    }
  }

  /*
   * ここまで処理が来るということは異常事態なので、positionとして
   * (-1,-1)を返す
   */
  returnPosition.x=-1;
  returnPosition.y=-1;
  return returnPosition;

}

/*
 * 指定されたactionに従って、座標を更新する
 * pursuit_position currentPosition: 
 * PredatorまたはPreyの位置座標を表すpursuit_position構造体
 * int action: 次の行動を表す文字(をint型で表したもの)
 * 'u','d','l','r','s'のどれか
 * 返り値:更新された位置座標を表すposition構造体
 */
pursuit_position movePP(pursuit_position currentPosition, int action){
  switch(action){
  case 'u': /* 上に移動 */
    currentPosition.y=currentPosition.y-1;
    break;
  case 'd': /* 下に移動 */
    currentPosition.y=currentPosition.y+1;
    break;
  case 'l': /* 左に移動 */
    currentPosition.x=currentPosition.x-1;
    break;
  case 'r': /* 右に移動 */
    currentPosition.x=currentPosition.x+1;
    break;
  case 's': /* その場に留まる */
    break;
  default:  /* 指定外の文字を指定した場合は's'と等価とみなす */
    break;
  }

  return (currentPosition);

}

/*
 * 移動した座標の妥当性を検討する
 * pursuit_position currentPosition: 
 * PredatorまたはPreyの位置座標を表すpursuit_position構造体
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * 返り値: 妥当ならばVALID(=1),そうでなければINVALID(=0)
 */
int evalPosition(pursuit_position currentPosition, 
		 int battleFieldArray[WORLD_SIZE][WORLD_SIZE]){
  
  /* 
   * ここで調べているのは,
   *   1. 指定された位置が、WORLD_SIZE*WORLD_SIZEの格子世界からはみ出して
   *      いないかどうか
   *   2. 指定された位置に障害物が存在していないかどうか
   */
  if (currentPosition.x >=0 && currentPosition.x < WORLD_SIZE
      && currentPosition.y >= 0 && currentPosition.y < WORLD_SIZE
      && battleFieldArray[currentPosition.y][currentPosition.x] != -1){
    return VALID;
  }
  else{
    return INVALID;
  }

}

/*
 * 格子世界を更新する
 * int battleFieldArray[WORLD_SIZE][WORLD_SIZE]
 *      : 格子世界を表すWORLD_SIZE*WORLD_SIZEの整数型2次元配列
 * int predatorAction: Predatorの行動を表す文字(をintで表したもの)
 * ('u','d','l','r','s'のどれか)
 * int preyAction: preyの行動を表す文字(をintで表したもの)
 * ('u','d','l','r','s'のどれか)
 * 返り値: 格子世界更新の結果Preyが捕獲されれば1、そうでなければ0
 */
int RenewWorld(int battleFieldArray[WORLD_SIZE][WORLD_SIZE],
	       int predatorAction, int preyAction){

  /* PredatorとPreyの位置座標を表すposition構造体 */
  pursuit_position predatorPosition, preyPosition;
  pursuit_position newPredatorPosition, newPreyPosition;

  /* PredatorとPreyの現在位置を得る */
  predatorPosition = getPredatorPosition(battleFieldArray);
  preyPosition = getPreyPosition(battleFieldArray);

  /* 移動後のPredatorとPreyの座標を得る */
  newPredatorPosition = movePP(predatorPosition, predatorAction);
  newPreyPosition = movePP(preyPosition, preyAction);

  /* battleFieldArrayから一旦PredatorとPreyを削除 */
  battleFieldArray[predatorPosition.y][predatorPosition.x] = 0;  
  battleFieldArray[preyPosition.y][preyPosition.x] = 0;

  /* もし移動後の座標値が妥当ならば */
  if (evalPosition(newPredatorPosition,battleFieldArray) == VALID){
    /* 新しい座標をpredatorPositionにコピー */
    predatorPosition.x = newPredatorPosition.x;
    predatorPosition.y = newPredatorPosition.y;    
  }

  /* もし移動後の座標値が妥当ならば */
  if (evalPosition(newPreyPosition,battleFieldArray) == VALID){
    /* 新しい座標をpreyPositionにコピー */    
    preyPosition.x = newPreyPosition.x;
    preyPosition.y = newPreyPosition.y;
  }

  /*
   * Prey(=VALUE_OF_PREY)とPredator(=VALUE_OF_PREDATOR)を再配置する 
   * この順番なのは、PredatorがPreyに上書きされるのはおかしいため
   */
  battleFieldArray[preyPosition.y][preyPosition.x] = VALUE_OF_PREY;
  battleFieldArray[predatorPosition.y][predatorPosition.x] = VALUE_OF_PREDATOR;

  /* 終了判定 */
  if (predatorPosition.x == preyPosition.x 
      && predatorPosition.y == preyPosition.y){
    return(1);
  }
  else{
    return(0);
  }

}

/*
 * 格子世界のコピーを行う
 * int copy[WORLD_SIZE][WORLD_SIZE]: コピー先
 * int original[WORLD_SIZE][WORLD_SIZE]: コピー元
 * 返り値: 0
 */
int copyWorld(int copy[WORLD_SIZE][WORLD_SIZE], 
	      int original[WORLD_SIZE][WORLD_SIZE]){
  int i,j;

  for (i=0; i<WORLD_SIZE; i++){
    for (j=0; j<WORLD_SIZE; j++){
      copy[i][j] = original[i][j];
    }
  }
  return(1);
}

#ifdef VERSION_1_2 
static int copy_array[WORLD_SIZE][WORLD_SIZE];     /* 格子世界(コピー) */

/*
 * Watch dog timer構造体の初期設定(スタート)
 */
static void setWatchdogTimer(){
  int status;

  /* 排他制御始まり */
  status=pthread_mutex_lock(&(wdtimer.mutex));
  if(status != 0){
    fprintf(stderr,"mutex lock fail!!\n");
    abort();
  }
  
  /* 本体 */
  wdtimer.count = WATCHDOG_COUNT_INIT;
  wdtimer.state = WATCHDOG_RUN; /* 時間の監視開始 */

  /* 排他制御終わり */
  status=pthread_mutex_unlock(&(wdtimer.mutex));
  if(status != 0){
    fprintf(stderr,"mutex unlock fail!!\n");
    abort();
  }
}

/*
 * Watch dog timerの解除
 * 1秒のWait用にも使用
 */
static void resetWatchdogTimer(){
  int status;

  /* 排他制御始まり */
  status=pthread_mutex_lock(&(wdtimer.mutex));
  if(status != 0){
    fprintf(stderr,"mutex lock fail!!\n");
    abort();
  }
  
  /* 本体 */
  wdtimer.state = WATCHDOG_STOP; /* 時間の監視終了 */
  
  /* 排他制御終わり */
  status=pthread_mutex_unlock(&(wdtimer.mutex));
  if(status != 0){
    fprintf(stderr,"mutex unlock fail!!\n");
    abort();
  }
}

/*
 * Watchdog timerスレッドの本体（関数)
 * void *arg:Prey/Predatorスレッドのthread IDへのポインタ
 */
static void *threadWatchdogTimer(void *arg){
  int status;
  pthread_t *tid = (pthread_t *) arg;
  int wdexit=0;

  setWatchdogTimer(); /* watchdog_t構造体の初期化 */

  /* 排他制御始まり */
  while(1){
    status=pthread_mutex_lock(&(wdtimer.mutex));
    if(status != 0){
      fprintf(stderr,"mutex lock fail!!\n");
      abort();
    }
  
    /* 本体 */
    if(wdtimer.count == 0){
      if(wdtimer.state == WATCHDOG_RUN){
	/* 時間内に終わらないPrey/Predatorスレッドをキャンセルする */
	status = pthread_cancel(*tid); 
      }
      /* ここは一時コメントアウト
      if(status != 0){
	fprintf(stderr,"pthread_cancel fail!!\n");
	abort();
      }
      */
      wdexit = 1; /* watchdog threadを終了するためのフラグ */
    }
    else {
      /* watch dog timerのカウンタを1引く */
      if(wdtimer.count>0){
	wdtimer.count--;
      }
      else{
	wdexit = 1; /* watchdog threadを終了するためのフラグ */
      }
    }

    /* 排他制御終わり */
    status=pthread_mutex_unlock(&(wdtimer.mutex));
    if(status != 0){
      fprintf(stderr,"mutex unlock fail!!\n");
      abort();
    }
    if(wdexit)
      {	
	/* watchdog threadを終了する */
	pthread_exit(0);
      }
    usleep(WATCHDOG_INTERVAL);
  }
}

/*
 * Prey スレッドの本体（関数)
 * void *arg:格子空間へのポインタ（int *)
 */
static void *threadPrey(void *arg)
{
  static int Action;
  int *ca = arg;
  int cancel_type;
  int status;

  /* 非同期キャンセルを可能にする */
  status = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,
				 &cancel_type);
  if(status != 0){
    fprintf(stderr,"pthread_setcanceltype fail!!\n");
    abort();
  }

  Prey(ca, &Action);  
  resetWatchdogTimer();  /* watchdog timerを終了する */
  pthread_exit(&Action); /* thread終了時の戻り値として次の動作を返す */
}

/*
 * Predator スレッドの本体（関数)
 * void *arg:格子空間へのポインタ（int *)
 */
static void *threadPredator(void *arg)
{
  static int Action;
  int *ca = arg;
  int cancel_type;
  int status;
  int i,j;

  /* 非同期キャンセルを可能にする */
  status = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,
				 &cancel_type);
  if(status != 0){
    fprintf(stderr,"pthread_setcanceltype fail!!\n");
    abort();
  }

  /*  usleep(1100*1000);*/
  /* for(;;) */
  Predator(ca, &Action);
  resetWatchdogTimer();  /* watchdog timerを終了する */
  pthread_exit(&Action); /* thread終了時の戻り値として次の動作を返す */
}

#endif /* VERSION_1_2 */
/*
 * Pursuit Gameのmain関数
 * 返り値: 正常終了なら0,異常終了なら1
 */

int main(int argc, char *argv[]){
  int i;
  int array[WORLD_SIZE][WORLD_SIZE];          /* 格子世界(オリジナル) */
#ifndef VERSION_1_2
  int copy_array[WORLD_SIZE][WORLD_SIZE];     /* 格子世界(コピー) */
#endif /* VERSION_1_2 */
  int gameStepNum;                            /* ゲームのステップ数 */
  int predatorAction, preyAction;             /* PredatorやPreyの動きを表す */

#ifdef VERSION_1_2
  int status;                                 /* pthread_*関数の戻り値用 */
  void *tret;                                 /* Prey/Predatorスレッド終了時の戻り値用 */
#endif /* VERSION_1_2 */

  /* 以下はオプション指定により変更される可能性のある変数 */
  int gameStepNum_Max;                        /* ゲームの最大ステップ数 */
  int displayWait;                            /* 表示更新間隔(マイクロ秒) */
  char worldFileName[WORLD_FILE_NAME_LENGTH]; /* 格子世界ファイル名 */

  /* 変数のデフォルト値を設定する */
  gameStepNum_Max = GAME_NUM;
  displayWait = DISPLAY_WAIT;
  strcpy(worldFileName, "battlefield.dat");

  /* 標準入力からのパラメータ処理 */
  for (i=1; i<argc; i++){
    /* "-s"(ステップ数指定)オプションの処理 */
    if (strcmp(argv[i], "-s")==0){              /* "-s"があったら */
      if (i+1 < argc){                          /* その次に入力があるか? */
	if (atoi(argv[i+1]) > 0){               /* 入力された値は妥当? */
	  gameStepNum_Max = atoi(argv[i+1]);    /* 全てOKのときの処理 */
	  i++;
	}
	else{
	  printf("Invalid input value!! Specify one positive number.\n");
	  /* 値が妥当でない場合 */
	  return(1);
	}
      }
      else{
	/* オプションの次に入力がない場合 */
	printf("Option error!! You must specify number of game step.\n");
	return(1);
      }
    }
    /* "-t"(表示間隔指定)オプションの処理 */
    else if(strcmp(argv[i], "-t")==0){         /* "-t"があったら */
      if (i+1 < argc){                         /* その次に入力があるか? */
	if (atoi(argv[i+1]) >= 0){              /* 入力された値は妥当か? */
	  displayWait = atoi(argv[i+1]);       /* 全てOKのときの処理 */
	  i++;
	}
	else{
	  printf("Invalid input value!! Specify one positive number.\n");
	  /* 値が妥当でない場合 */
	  return(1);
	}
      }
      else{
	/* オプションの次に入力がない場合 */
	printf("Option error!! you must specify interval time of redraw.\n");
	return(1);
      }
    }
    /* "-w"(格子世界ファイル名指定)オプションの処理 */
    else if(strcmp(argv[i], "-w")==0){          /* "-w"があったら */
      if (i+1 < argc){                          /* その次に入力があるか? */
	/* 全てOKのときの処理 */
	strncpy(worldFileName, argv[i+1], WORLD_FILE_NAME_LENGTH);
	i++;
      }
      else{
	/* オプションの次に入力がない場合 */
	printf("Option error!! You must specify world file name.\n");
	return(1);
      }
    }
    else {
      printf("Invalid option: available options are:\n");
      printf("-s: Specify number of game step. ex: \"-s 10\"\n");
      printf("-t: Specify interval  time of redraw. ex: \"-t 100000\"\n");
      printf("-w: Specify world file name. ex: \"-w battlefield2.dat\"\n");
      return(1);
    }
  }

  /* BattleFieldの読み込み */
  if (ReadInitConf(array,worldFileName) == 1){
    fprintf(stderr,"File read failed!!\n");
    return(1);
  }

  /* 表示のクリア */
  system("clear");

  /* 初期状態の表示 */
  PrintWorld(array,0);

#ifdef VERSION_1_2
  /* 並列実行するスレッド数をセットする */
  pthread_setconcurrency(3);
#endif /* VERSION_1_2 */

  /* ゲームスタート */
  for (gameStepNum=1; gameStepNum<= gameStepNum_Max; gameStepNum++){

#ifdef VERSION_1_2 /* 時間監視バージョン */
    /* Predatorの行動 */
    copyWorld(copy_array,array);

    /* Predator threadを作る */
    status = pthread_create(&player_tid, NULL, threadPredator, (void *)copy_array);
    if(status != 0){
      fprintf(stderr,"pthread_create fail!!: %s\n", strerror(status));
      abort();
    }

    /* watch dog timer threadを作る */
    status = pthread_create(&wd_tid, NULL, threadWatchdogTimer, (void *)&player_tid);
    if(status != 0){
      fprintf(stderr,"pthread_create fail!!: %s\n", strerror(status));
      abort();
    }

    /* Predator threadの終了を待つ．終了戻り値はtret */
    status = pthread_join(player_tid, &tret);
    if(status != 0){
      fprintf(stderr,"pthread_join fail!!: %s\n", strerror(status));
      abort();
    }

    /* Predator threadがキャンセルされたか確認 */
    if(tret == (void *)PTHREAD_CANCELED){
      predatorAction = 's';
    }
    else{
      predatorAction = *((int *)tret);
    }

    /* watch dog threadの終了を待つ．終了戻り値はtret */
    status = pthread_join(wd_tid, &tret);
    if(status != 0){
      fprintf(stderr,"pthread_join fail!!: %s\n", strerror(status));
      abort();
    }

    /* Preyの行動(2回動いて1回休む) */
    if (gameStepNum % 3 != 0){ 
      copyWorld(copy_array,array);

      /* Prey threadを作る */
      status = pthread_create(&player_tid, NULL, threadPrey, (void *)copy_array);
      if(status != 0){
	fprintf(stderr,"pthread_create fail!!: %s\n", strerror(status));
	abort();
      }

      /* watch dog timer threadを作る */
      status = pthread_create(&wd_tid, NULL, threadWatchdogTimer, (void *)&player_tid);
      if(status != 0){
	fprintf(stderr,"pthread_create fail!!: %s\n", strerror(status));
	abort();
      }

      /* Prey threadの終了を待つ．終了戻り値はtret */
      status = pthread_join(player_tid, &tret);
      if(status != 0){
	fprintf(stderr,"pthread_join fail!!: %s\n", strerror(status));
	abort();
      }

      /* Prey threadがキャンセルされたか確認 */
      if(tret == (void *)PTHREAD_CANCELED){
	preyAction = 's';
      }
      else{
	preyAction = *((int *)tret);
      }

      /* watch dog threadの終了を待つ．終了戻り値はtret */
      status = pthread_join(wd_tid, &tret);
      if(status != 0){
	fprintf(stderr,"pthread_join fail!!: %s\n", strerror(status));
	abort();
	
      }
    }
    else{
      preyAction = 's';
      usleep(WATCHDOG_COUNT_INIT*WATCHDOG_INTERVAL);
    }
#else /* VERSION_1_2 */
    /* Predatorの行動 */
    copyWorld(copy_array,array);
    Predator(copy_array, &predatorAction);

    /* Preyの行動(2回動いて1回休む) */
    if (gameStepNum % 3 != 0){
      copyWorld(copy_array,array);
      Prey(copy_array,&preyAction);      
    }
    else{
      preyAction = 's';
    }
#endif /* VERSION_1_2 */

    /* 表示のウェイト */
    usleep(displayWait);

    /*
     * それぞれの行動に合わせて格子世界を更新
     * RenewWorldの帰り値が0ならば継続
     */
    if (RenewWorld(array,predatorAction, preyAction) == 0){
      system("clear");
      PrintWorld(array, gameStepNum);
    }
    /* そうでなければ(1ならば)終了 */
    else{
      system("clear");
      PrintWorld(array, gameStepNum);      
      printf("Game Over!! Winner Predator (%d clock)!!\n",gameStepNum);
      return(0);
    }

  } 

  /* Preyが最後まで逃げ切ったときのメッセージ出力 */
  printf("Game Over!! Winner Prey!!\n");
  return(0);
}

