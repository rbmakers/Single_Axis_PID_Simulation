
<img width="1093" height="594" alt="螢幕錄影 2026年八月06日 10時03分01秒" src="https://github.com/user-attachments/assets/59a6349d-5fe4-4d5d-a007-f7fb5eac061a" />




這份模擬程式是一個基於網頁（HTML5 Canvas）的**單軸 PID 控制與反應頻率實驗台**。以下數學式詳細梳理該模擬程式背後的物理模型、離散 PID 控制器架構以及數位控制的時間同步機制：

---

### 一、 物理系統與旋轉運動方程式

模擬中的旋轉機臂可視為一個受力矩作用的單軸旋轉系統。根據牛頓第二運動定律（旋轉版），其角加速度 $\alpha$ 與受到的總力矩 $\tau_{\text{total}}$ 關係為：

$$J \alpha = \tau_{\text{total}}$$

其中：
* $J$ 為**轉動慣量**（單位： $$\text{kg}\cdot\text{m}^2$$ ），程式中預設設為 $J = 0.025$。
* $\alpha = \frac{d\omega}{dt}$ 為角加速度（單位： $$\text{rad}/\text{s}^2$$ ）。
* $\omega$ 為當前的**角速度**（單位： $$\text{rad}/\text{s}$$ ）。
* 角度 $\theta$ 的變化率即為角速度：
  $$\frac{d\theta}{dt} = \omega$$

#### 總力矩 $\tau_{\text{total}}$ 的組成：
系統在每個瞬間受到的總力矩包含控制器輸出的主動控制力矩 $\tau_{\text{control}}$ 以及系統本身的微小自然阻尼（與角速度成正比）：

$$\tau_{\text{total}} = \tau_{\text{control}} - b \omega$$

* $b$ 為**自然阻尼係數**（程式中預設 `base_b = 0.005`）。

---

### 二、 離散 PID 控制器數學模型

在數位控制系統中（如模擬 RP2350 微控制器的執行緒）[cite: 1]，PID 計算是以固定的控制週期 $T_s = \frac{1}{f_{\text{control}}}$ 進行。

1. **誤差計算（Error）：**
   設目標角度為 $\theta_{\text{target}}$（由度數轉換為弳度），當前角度為 $\theta$，則離散時間點 $k$ 的誤差 $e[k]$ 為：
   $$e[k] = \theta_{\text{target}} - \theta[k]$$

2. **比例項（Proportional, $P$）：**
   $$P[k] = K_p \cdot e[k]$$
   * $K_p$ 相當於彈簧剛度[cite: 1]。

3. **積分項（Integral, $I$）：**
   採用數值積分（矩形法），並具有防飽和機制（Anti-windup，限制在 $[-2.0, 2.0]$ 之間）：
   $$\text{Integral}[k] = \text{Integral}[k-1] + e[k] \cdot T_s$$
   $$\text{Integral}[k] = \min(\max(\text{Integral}[k], -2.0), 2.0)$$
   $$I[k] = K_i \cdot \text{Integral}[k]$$

4. **微分項（Derivative, $D$）：**
   採用標準離散後向差分（Backward Difference）：
   $$D[k] = K_d \cdot \frac{e[k] - e[k-1]}{T_s}$$
   * 此項扮演物理阻尼的角色[cite: 1]。

5. **控制器總輸出（力矩）：**
   $$\tau_{\text{control}}[k] = P[k] + I[k] + D[k]$$
   此控制力矩會維持定值，直到下一個控制週期到來。

---

### 三、 時間同步與多速率架構 (Multi-rate Architecture)

模擬程式的核心亮點在於分離了**「控制運算頻率 ($f_{\text{control}}$)」**與**「繪圖渲染更新率 ($60\,\text{fps}$ 畫布動畫)」**。其運作機制如下：

* 藉由時間累積器（`control_accumulator`）計算實際流逝的時間 $\Delta t$：
  $$\text{accumulator} \leftarrow \text{accumulator} + \Delta t$$
* 當累積時間大於或等於離散控制週期時（即 $\text{accumulator} \ge \frac{1}{f_{\text{control}}}$）：
  * 觸發一次上述的**離散 PID 核心運算**。
  * 累積器扣除一個週期： $$\text{accumulator} \leftarrow \text{accumulator} - \frac{1}{f_{\text{control}}}$$ 。
* 在每一個動畫畫格（Animation Frame）中，物理世界則以高頻率的 $\Delta t$ 進行歐拉積分更新（Euler Integration）：
  $$\omega^{(n+1)} = \omega^{(n)} + \left(\frac{\tau_{\text{control}} - b \omega^{(n)}}{J}\right) \Delta t$$
  $$\theta^{(n+1)} = \theta^{(n)} + \omega^{(n)} \Delta t$$

這樣的數學與程式架構，能真實呈現當**控制反應頻率（ $$f_{\text{control}}$$ ）**過低時，因控制更新延遲所造成的系統震盪與發散現象[cite: 1]。
