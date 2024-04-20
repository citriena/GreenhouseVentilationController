﻿# 農業用温室側窓自動制御Arduinoスケッチ
citriena
2024年04月版

https://github.com/citriena/GreenhouseVentilationController


## 概要
これは，温室の温度制御用の側窓開閉モーターをを自動で制御するためのArduinoスケッチです．私が使っているのは巻上／巻下式側窓なので以下の説明では巻上／巻下を使用しますが、はね上げ式や引き戸式の側窓や天窓でも制御可能と思います。

使用状況は、ブログ記事の[自作温室側窓自動開閉装置](https://citriena.seesaa.net/article/Vent_Controller.html)を参照ください．
## 主な機能
* 設定した温度，感度に応じて側窓開閉モーターを制御し，温室内の温度を調節する．
  * 手動での側窓の全開，全閉，停止，および除湿のための一定時間全開等可能
* 設定した最低温度に応じて加温機を起動／停止する．
  * 応差（ヒステリシス）の設定も可能
* 測定した気温をmicroSDカードに記録可能
  * 加えて加温機の起動／停止時刻の記録も行う．

## 必要機材等
### Arduino UNO R3
* おそらくR4でも可

### I2C LCD keypad シールド
* Adafruit I2C Controlled + Keypad Shield Kit for 16x2 LCD，または互換品

### 自作専用シールド
* サーミスタ接続端子，加温機制御用出力端子，降雨センサ接続端子，microSDカードモジュール，RTCモジュール実装
* 一番上に装着する必要があるI2C LCD keypadシールドは，製品によってはI2C通信にA4, A5ピンを使わずI2Cピンだけを使っているものがある。しかし，Cytron SHIELD-MD10モータードライバシールドにはI2Cピンが実装されていないので，この自作シールドでA4, A5ピンとI2Cピンを接続している．このため，必ず本シールドをI2C LCD keypad シールドの直下に重ねる．でないとI2C LCD keypad シールドが動作しない．
* microSDカードモジュール，RTCモジュールの実装は必須ではない．非実装の場合は，コンパイル設定を変更してコンパイルする．

### モータードライバシールド
* 使用しているのは，Cytron SHIELD-MD10
* DIR: 7, PWM: 6 に設定する．
* 定格10A(ピーク15A 10秒以内)なので，それ以下で使用する．一般的な側窓モーター２台（ハウス両側）であればおそらく問題ないと思うが，確認すること．モーターの消費電流は負荷によって変わるので，無負荷での計測値は当てにならない．また，モーター始動時は一般的に動作時の数倍の電流が流れるので，それも考慮して十分余裕を見ること．複数のモーターを使用する場合は合計がこれを超えないようにする． 制限を超える場合は，モーター毎に個別のモータードライバを使う等して対応する．
* 正転，逆転が可能であれば他のモータードライバでも可．ただし，スケッチの修正が必要と共に，使用するArduinoのピンに注意

### サーミスタ温度センサ
* こちらで使用しているのは秋月電子通商で扱っているSEMITEC株式会社103AT-11，もしくは103AT-2である．これらと特性が異なる場合はデータシートを参照してその特性をArduino用スケッチに設定してコンパイルする必要がある．必要となるのは0℃，25℃，50℃時のサーミスタの抵抗値である．温度は多少異なっても良いが，実際に使用する温度域を含んでいること．
* 詳細はSHthermistorライブラリの説明参照
* センサはなるべく強制通風筒を使って設置する．

### 側窓開閉用モーター
* 電源DC24Vで，+-を入れ替えることで回転が反転するモーターを使う．モータードライバの定格以内のものとする．
* 上下のリミッタ設定ができるもの
* こちらではAmazonで購入した物を使っている．必要に応じリフターを使う等して設置する．

### 側窓モーター用電源
* 側窓モーターを駆動可能な容量のものを使う．モーター始動時は定格の数倍の電流が流れることもあるので，十分余裕を見ること．モーター動作電流の定格では足りていても起動時電流が電源の定格を超えていると，電源の保護回路が作動してモーターが動作しなくなることがある．

### Arduino用電源
* 12V 0.5A以上あればよい．加温機の制御にリレーを使う場合は，それも考慮した電源とする．

### リレーおよび加温機（加温機を使用する場合）
* Arduinoの電源と同じ電圧に対応したリレーを使う．
* 加温機の出力制限を行う場合はSSR（半導体リレー）を使う必要があるとともに，PWMによる出力制御に対応した加温機（電熱器等）を使用する必要がある．

### 降雨センサ（降雨時に側窓閉鎖する場合）
* 降雨を検知すると回路を短絡できるもの．
 * 降雨センサ入力がGNDに接続される（LOWになる）と降雨と判定する．
 * 逆であればスケッチを修正するかリレー等で反転させる．

## LCD表示
基本表示のみ説明する．設定表示は基本操作の設定項目参照のこと．

基本表示例：


| SP : 25. 0 ﾟ  AUTO 　 ^|
| - |
|TP : 28. 0 ﾟ 　18s　 5 ﾟ |


基本表示内容：

| 設定温度 | 動作モード | モーター動作 |
| - | - | - |
| 測定温度 | 巻上カウンタ | 加温設定温度 |

* 設定温度：制御設定温度
* 測定温度：実際に測定した温度．1秒ごとに測定，更新される．
* 動作モード
  * AUTO: 自動制御モード
  * CLOSE: 全閉モード
  * OPEN: 全開モード
  * DEHUMID: 除湿モード
  * RESET: 側窓リセットモード
* 側窓モーター動作
  * ^ : 巻上動作中
  * v : 巻下動作中
  * \- : 停止中
  * →: 巻上上限
  * ←: 巻下下限
* 巻上カウンタ：現在の巻上カウンタ値（秒）
  * 0で側窓が下がりきった状態
* 加温設定温度：加温動作中は１秒間隔で点滅

## 基本操作
LCDの表示には現在の動作状況画面（基本表示），および各項目設定画面がある．キーにより表示項目の移動，設定値の変更，確定を行なう．基本的な操作は以下の通りである．
* [左][右]キーで表示項目移動
* [上][下]キーで設定値変更
  * 基本画面では最低最高温度の表示（最大５日間）
* [SELECT]キーで設定確定
  * 設定変更内容を不揮発性メモリに保存し，再起動時は設定した内容で動作再開する．
  * [SELECT]キーを押す前に[左][右]キーで表示項目を移動したり，30秒間無操作で基本画面に移行すると設定変更されない．
* キーを30秒間操作しないと基本表示になる．
* 起動時に何かキーを押しておくと設定内容を破棄し，設定が初期値になる．

### 設定項目
LCDに表示される設定項目について説明する．

#### MODE
 * 内容：側窓動作モード設定
 * 設定：
   * AUTO: 設定温度等に従い，側窓の開閉を自動制御する．
   * CLOSE: 側窓を閉じる．
   * OPEN: 側窓を開く．
   * DEHUMID: 設定時間だけ側窓を全開後，自動制御に移行する．
   * RESET: 側窓を一旦全閉し，その後自動制御に移行する．
     * RTCを実装している場合は，側窓のズレを解消するために日付が変更になるタイミングで自動的にRESETを実行する．
 * 説明：側窓の動作モードを設定する。再起動時は一旦リセット（全閉）後に前回の動作モードで動作開始する．

#### SP:
 * 内容：側窓制御温度設定
 * 設定範囲：0-40（℃）
 * 初期値：25
 * 説明：この設定温度に従って側窓の開閉動作を制御する．

#### HEAT:
 * 内容：加温制御温度
 * 設定範囲：0-30（℃）
 * 初期値：5
 * 説明：この設定温度以下になると加温機を動作させる．基本的にはON/OFF制御である．応差（ヒステリシス）動作可能．デフォルトでは応差動作無．応差感度の設定はスケッチ内で行うため，設定変更には再コンパイルが必要．なお，Arduinoの加温出力制限入力ピン（デフォルト3番ピン）をLOW（GNDと接続）にするとPWMにより加温出力を調整できる（ただし，PWMに対応した加温機のみ）．私の都合により実装した機能で，必要とする方はほとんどいないでしょうが．

#### SENS:
 * 内容：側窓制御時の感度
 * 設定範囲：0.0-5.0（℃）
 * 初期値：1.0
 * 説明：温度が設定温度±感度内になるように制御する．この範囲内では側窓モーターを制御しない．設定値が小さい方が温度をより設定値に近く制御できるが，その分側窓モーターが頻繁に動作することになる．

#### MAX WIND T:
 * 内容：累積巻上時間上限
 * 設定範囲：0-600（秒）
 * 初期値：150
 * 説明：累積巻上時間が上限に達したら巻上を停止する．使っているモーターによっては、時間だけで制御していると停止位置がずれてくる．このため，実際の巻上上限は巻上モーターのリミッタで設定し，この累積巻上上限はそれよりも少し多めに設定するのが良い．

#### WINDING TIME:
 * 内容：１回の巻上，巻下時間
 * 設定範囲：1-120（秒）
 * 初期値：10
 * 説明：温度制御では巻上／巻下と休止を交互に繰り返して温度を制御する．その巻上／巻下の１回の動作時間．基本はここで設定した時間であるが，設定温度から急速に離れつつある場合はこの２倍で動作することもある．

#### PAUSE TIME:
 * 内容：温度自動制御時における側窓モーターの休止時間
 * 設定範囲：0-120（秒）
 * 初期値：30
 * 説明：温度制御では側窓モーターの巻上／巻下と休止を交互に繰り返して温度を制御する．その１回の休止時間．休止時間は連続する場合もある．

#### EXTRA REWIND:
 * 内容：巻下追加時間／初回巻上追加時間
 * 設定範囲：0-120（秒）
 * 初期値：20
 * 説明：側窓巻上カウンタが０になった後追加で巻下げる時間．側窓モーターの巻上／巻下の不整合により巻上カウンタが０になっても最下端に達しない場合がある．そのような場合でも最下端に達するように，全閉時にここで設定した時間だけ追加で巻下げる．全閉状態から巻き上げる場合に追加する時間も兼ねているので，全閉状態から側窓が実際に開き始めるまでの時間を設定する．これにより，全閉状態からの巻上時における側窓が実際に開き始めるまでの無駄な時間を減らしている．
   * 外気環境によっては，これでも完全には閉まらなくなる事がある。このためRTCを実装している場合は日付が変わるときに側窓リセットを行うようにしている．
   * 必要以上に巻き下げないように，側窓モーターの下端リミッタを設定すること．側窓モーターに下端リミッタがない場合は本設定を必ず0にする．

#### DEHUMID TIME:
 * 内容：除湿時間
 * 設定範囲：0-60（分）
 * 初期値：5
 * 説明：側窓を全開にして換気／除湿を行う時間．除湿時間終了後は自動制御モードに移行する．

#### OFFSET TEMP:
 * 内容：温度補正値
 * 設定範囲：-5.0～5.0（℃）
 * 初期値：0
 * 説明：計測値にこの補正値を加えた値が測定温度となる．

#### LOG MODE:
 *内容：データ記録設定
 * 設定範囲：0, 1, 2, 5, 10（分）；0は記録しない
 * 初期値：0
 * 説明：microSDカードに測定温度データ記録する間隔を設定する．記録開始するのは設定時からではなく，時刻の0分を基準にした時間となる．記録する測定温度は前回の記録後毎分毎に測定したデータの平均値である．なお，データ記録する設定にすると，加温のON/OFF時間も記録する． 記録中のmicroSDカードを取り出す場合は記録設定を0にしてから行う．書き込む回数を減らすためバッファ処理を行っているので、この操作で未保存のデータを全てmicroSDカードに書き出す．

#### 時計表示，設定
 * 時計表示，および設定を行う．[SELECT]を押した時点を0秒として時計を設定する．

## コンパイル設定項目
コンパイル時に設定する項目（一部）．将来的にキーパッドで設定変更できるようにする可能性あり．
### #define POWER_RATE
 * 内容：加温出力制限時の出力
 * 設定範囲：0～100（%）
 * 初期値：70
 * 説明：Arduinoの加温出力制限入力ピン（デフォルト3番ピン）をLOW（GNDと接続）にするとPWMにより加温出力を調整する．このため，電熱線等のPWM制御可能な加温機以外では使用不可．また，実際の出力制御にはSSR（半導体リレー）などPWMに対応した装置で加温機を制御することが必要である．当然ながら機械式リレーでは使用できない．

### #define HYSTERESIS
 * 内容：加温動作時の応差温度（ヒステリシス）
 * 設定範囲：0～5程度（特に制限は無い）
 * 初期値：0
 * 説明：加温動作時の応差温度を設定する．加温設定温度以下になると加温開始するが，加温停止は加温設定温度＋応差温度以上になったときである．

### #define ELECTRICITY_LIMIT
 * 内容：側窓モーターと加温機の同時動作禁止
 * 初期値：定義
 * 説明：加温機と側窓モーターが同時に動作すると電力の上限を超えてブレーカーが落ちる可能性がある場合，本設定を行うと側窓モーター動作時には加温機を停止する．一般的には加温機動作時は側窓全閉なので通常動作で本設定が実行される可能性は低いが、巻上／巻下の不整合によって全閉になっていない場合、夜間に自動リセットで側窓モーターが動作する可能性がある。これでブレーカーが落ちて停止することがないように本設定を実装している．

## 側窓制御動作アルゴリズム
温度による側窓制御のアルゴリズムは以下の通りである．側窓モーターの動作をなるべく減らして制御することを目指しており，PID制御のような高度なフィードバック制御は行っていない．このため，動作開始時はフィードバック制御と比べると安定までに時間が多少余計にかかるかもしれない．実際に使った印象は，意図したとおり側窓モーターの動作が少なく，かつ設定温度から極端にずれることもなく，側窓制御として十分に実用的と判断している．

1. 温度測定のタイミング：休止モード終了後
1. 動作アルゴリズム
   1. 設定温度 ±感度 以内だったら休止モード
   1. それ以外の場合は前回と今回の温度測定値を比較し
      1. 設定温度に近づいていたら休止モード
      1. 変化無，もしくは設定温度から離れていたら
         1. 設定温度よりも低かったら巻下
         1. 設定温度よりも高かったら巻上
         1. 前回測定値よりも0.5℃以上離れていたら巻上／巻下時間２倍
         1. 巻上／巻下後は休止モード
1. １回の巻上／巻下時間：10秒（変更可）
1. １回の休止時間：30秒（変更可）

## Arduinoの入出力ピン使用状況

Arduino の入出力ピン使用状況は以下の通り．
未使用の入出力ピンがあるので，機能追加も可能

```
D00: Serial
D01: Serial
D02: サーミスタ加電圧（測定時HIGH）
D03: 加温機出力制限入力（出力制限時LOW）
D04: Ethernetシールドを使うときは，これがmicroSDのSPI(SS)
D05: 加温機動作出力（動作時HIGH）
D06: モータードライバ制御出力（PWM）
D07: モータードライバ制御出力（DIR）
D08: 降雨センサ入力（LOWで全閉）
D09:
D10: SPI(SS)
D11: SPI(MOSI)
D12: SPI(MISO)
D13: SPI(SCK)
A00:
A01:
A02: サーミスタ測定入力
A03
A04: I2C(SDA)
A05: I2C(SCL)
```

## 本資料等について
本資料および当該スケッチは無保証です．本資料等を使用することによって生じる，いかなる直接的・間接的損害について著作者はいかなる責任，サポート義務を負いません．

スケッチにバグがありましたら以下まで連絡いただけると幸いです．可能な限り対応したいと思います．本資料の間違い，不適切な内容についても同様です．機能の要望等の意見についても適切な内容であれば実装に努力したいと思いますが，対応できるかどうかはわかりません．

初版：2023年12月　最終改定：2024年04月17日
citriena@yahoo.co.jp
