# drone
Raspberry Pi Zero で自律飛行ドローン作成

--- 

## master branch


![image](https://user-images.githubusercontent.com/12773136/43676221-f3bdefaa-9827-11e8-811b-cb826ebf8dd1.jpg)

![image](https://user-images.githubusercontent.com/12773136/43676231-18f4e5a8-9828-11e8-921f-6428db3d63ec.PNG)


![image](https://user-images.githubusercontent.com/12773136/43676234-212db9b6-9828-11e8-9b9b-e0dd015f0483.PNG)


### Reference

https://qiita.com/shigeru-yokochi/items/0a0a4f06cd8dec553159

https://www.yokochi.jp/categories/drone/

https://www.youtube.com/watch?v=y1wnsGAwrUc

---

## betafpv branch (master -> betafpv)

![image](https://user-images.githubusercontent.com/12773136/86593366-08cd5a80-bfd0-11ea-9f26-9ea41cd77b76.jpeg)

![image](https://user-images.githubusercontent.com/12773136/86593390-14b91c80-bfd0-11ea-9557-1d50f6b7ce72.jpeg)


![image](https://user-images.githubusercontent.com/12773136/86593411-1edb1b00-bfd0-11ea-994b-16ae68178bc9.jpeg)

### Reference

https://qiita.com/shigeru-yokochi/items/7f74ccf200a57388e241

https://www.yokochi.jp/post/betafpv-f4-2s-aio2/

---

## VL53L0X_multi_test branch (master -> betafpv -> VL53L0X_multi_test)

![image](https://user-images.githubusercontent.com/12773136/87627932-d0b1de80-c76a-11ea-9244-5b19050b3075.jpg)

![image](https://user-images.githubusercontent.com/12773136/87628047-08b92180-c76b-11ea-83c4-acbfa9b98c95.png)

### Reference

https://qiita.com/shigeru-yokochi/items/f996d495effdf6dea3fc

---

## QAV210 branch (master -> QAV210)


![20200728_212705](https://user-images.githubusercontent.com/12773136/88665508-af150780-d119-11ea-9215-3024a63091c2.jpg)

![構成図](https://user-images.githubusercontent.com/12773136/89098291-251eb480-d421-11ea-8302-836958e13131.png)


![グラフ](https://user-images.githubusercontent.com/12773136/89097762-be979780-d41c-11ea-82b3-c3a84aab72ab.png)


[![youtube movie](http://i.ytimg.com/vi/JrEqpENNYcE/hqdefault.jpg)](https://www.youtube.com/watch?v=JrEqpENNYcE)


### 高度を維持する方法

以下の状態とイベントに分けて処理します

|  event\status  |  1 <br>停止中 | 2<br>設定高度以下を上昇中 | 3<br>設定高度を超えて上昇中 | 4<br>設定高度以下まで降下待ち | 5<br>ランディング中 |
| ---- | ---- | ---- | ---- | ---- | ---- |
|  1 <br>開始 |  TAKEOFF_POWER<br>status=2  | | | | |
|  2 <br> N/A |    | ||| |
|  3 <br>現在高度は最大値超え |    | LANDING_POWER<br>status=3|
|  4 <br>下降検知 |   |power+10 |TAKEOFF_POWER<br>status=4|設定高度未満の場合、TAKEOFF_POWER<br>status=2|
|  5 <br>ランディング |    |LANDING_POWER<br>status=5 |LANDING_POWER<br>status=5 |LANDING_POWER<br>status=5 |

### 前後左右に衝突しない方法

- 前後(左右)の距離を測り閾値を超えたら逆方向へ移動する
- 前後(左右)両方の距離が閾値を超えている場合はエラーとする（今回は何も処理しない）
- 逆方向へ移動する出力は固定とする（とりあえず）

---

## betafpv_VL53L0X_single branch (master -> betafpv -> betafpv_VL53L0X_single)

![image](https://user-images.githubusercontent.com/12773136/87650959-66f6fc00-c78d-11ea-926d-ed318a7372e5.jpg)

### 高度を維持する方法

以下の状態とイベントに分けて処理します

|  event\status  |  1 <br>停止中 | 2<br>設定高度以下を上昇中 | 3<br>設定高度を超えて上昇中 | 4<br>設定高度以下まで降下待ち | 5<br>ランディング中 |
| ---- | ---- | ---- | ---- | ---- | ---- |
|  1 <br>開始 |  TAKEOFF_POWER<br>status=2  | | | | |
|  2 <br> N/A |    | ||| |
|  3 <br>現在高度は最大値超え |    | LANDING_POWER<br>status=3|
|  4 <br>下降検知 |   |power+10 |TAKEOFF_POWER<br>status=4|設定高度未満の場合、TAKEOFF_POWER<br>status=2|
|  5 <br>ランディング |    |LANDING_POWER<br>status=5 |LANDING_POWER<br>status=5 |LANDING_POWER<br>status=5 |

### 姿勢制御方法

MPU6050より機体の傾斜を検出してrollとpitchの出力値を変更している（debug中）

### PID制御

未実装

### BLE制御

未実装

### 衝突防止方法

未実装

### ジャイロ

未実装

### 気圧センサー

未実装

### Reference

https://www.yokochi.jp/post/betafpv-f4-2s-aio2/
