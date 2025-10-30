## top_base_notify

top_base_notify主要為捲揚機提供移動訊號
根據捲揚機的架設長度調整內部參數，一旦超過則開始往下移動
得到rectangles_detect的/World_Coordinates則停止發出移動訊號
壓到捲揚機的敲擊復歸極限開關/move_hit_initialized則會根據當前主要往左或往右發出多次訊號




## 版本動態

* v110
  * 功能:
    * 使用/move2給捲揚機移動訊號
    * /move可以利用uid確保當前訊號是否為最新






## 版本bug




## 注意

    src後的檔案不能與前面檔案名字相符，否則編譯報錯
    
