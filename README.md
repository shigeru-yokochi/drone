# drone
Raspberry Pi Zero で自律飛行ドローン作成

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

### Reference

https://www.yokochi.jp/post/betafpv-f4-2s-aio2/
